// Production embedded FujiNet-NIO SIO adapter regression.
//
// The test deliberately drives ATDeviceFujiNetNIO through IATDeviceSIO, not
// through a copied native-NIO peer.  FakeSIO records only Altirra's real SIO
// scheduling calls; all Q/W/P/R decisions and packet routing remain in the
// production AtariSioNioFrontend embedded by the device.

#include <stdafx.h>
#include <at/atcore/device.h>
#include <at/atcore/devicesio.h>
#include <at/atcore/propertyset.h>

#include "fujinet/io/atari_sio_nio_frontend.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

void ATCreateDeviceFujiNetNIO(const ATPropertySet& pset, IATDevice **dev);

namespace {

[[noreturn]] void Fail(const char *message) {
	std::fprintf(stderr, "FAIL: %s\n", message);
	std::exit(1);
}

void Check(bool condition, const char *message) {
	if (!condition)
		Fail(message);
}

class FakeSIO final : public IATDeviceSIOInterface {
public:
	int AddRef() override { return ++mRefs; }
	int Release() override {
		const int refs = --mRefs;
		if (!refs)
			delete this;
		return refs;
	}

	uint32 GetAccelTimeSkew() const override { return 0; }
	uint64 GetCommandQueueTime() const override { return 0; }
	uint64 GetCommandFrameEndTime() const override { return 0; }
	uint64 GetCommandDeassertTime() const override { return 0; }
	bool IsActiveCommandAccelerated() const override { return false; }
	void SetCommandDeassertCheckEnabled(bool) override {}
	void SetCommandTruncationEnabled(bool) override {}
	void SetReceiveByteTimeout(uint64) override {}
	void SetReceiveFrameTimeout(uint64) override {}

	void BeginCommand() override { events.emplace_back("begin"); }
	void SendData(const void *data, uint32 len, bool addChecksum) override {
		events.emplace_back("data");
		const auto *bytes = static_cast<const std::uint8_t *>(data);
		transmitted.insert(transmitted.end(), bytes, bytes + len);
		if (addChecksum)
			transmitted.push_back(fujinet::io::AtariSioNioFrontend::checksum(bytes, len));
	}
	void SendACK() override { events.emplace_back("A"); transmitted.push_back('A'); }
	void SendNAK() override { events.emplace_back("N"); transmitted.push_back('N'); }
	void SendComplete(bool) override { events.emplace_back("C"); transmitted.push_back('C'); }
	void SendError(bool) override { events.emplace_back("E"); transmitted.push_back('E'); }
	void ReceiveData(uint32 id, uint32 len, bool autoProtocol) override {
		events.emplace_back("receive");
		receiveId = id;
		receiveLength = len;
		receiveAutoProtocol = autoProtocol;
	}
	void SetTransferRate(uint32, uint32) override {}
	void SetSynchronousTransmit(bool) override {}
	void Delay(uint32) override { events.emplace_back("delay"); }
	void InsertFence(uint32 id) override { events.emplace_back("fence"); fenceId = id; }
	void FlushQueue() override {}
	void EndCommand() override { events.emplace_back("end"); }
	void HandleCommand(const void *, uint32, bool) override { Fail("adapter used HandleCommand"); }
	void SaveActiveCommandState(IATObjectState **) const override {}
	void LoadActiveCommandState(IATObjectState *) override {}

	void Clear() {
		events.clear();
		transmitted.clear();
		receiveId = 0;
		receiveLength = 0;
		receiveAutoProtocol = true;
		fenceId = 0;
	}

	std::vector<std::string> events;
	std::vector<std::uint8_t> transmitted;
	uint32 receiveId{};
	uint32 receiveLength{};
	bool receiveAutoProtocol{};
	uint32 fenceId{};

private:
	int mRefs{};
};

class FakeSIOManager final : public IATDeviceSIOManager {
public:
	~FakeSIOManager() = default;
	vdrefptr<IATDeviceSIOInterface> AddDevice(IATDeviceSIO *) override {
		return vdrefptr<IATDeviceSIOInterface>(mSIO);
	}
	sint32 GetHighSpeedIndex() const override { return -1; }
	uint32 GetCyclesPerBitRecv() const override { return 0; }
	uint32 GetRecvResetCounter() const override { return 0; }
	uint32 GetCyclesPerBitSend() const override { return 0; }
	uint32 GetCyclesPerBitBiClock() const override { return 0; }
	void AddRawDevice(IATDeviceRawSIO *) override {}
	void RemoveRawDevice(IATDeviceRawSIO *) override {}
	void SendRawByte(uint8, uint32, bool, bool, bool) override {}
	void SetRawInput(bool) override {}
	bool IsSIOCommandAsserted() const override { return false; }
	bool IsSIOMotorAsserted() const override { return false; }
	bool IsSIOReadyAsserted() const override { return true; }
	bool IsSIOForceBreakAsserted() const override { return false; }
	void SetSIOInterrupt(IATDeviceRawSIO *, bool) override {}
	void SetSIOProceed(IATDeviceRawSIO *, bool) override {}
	void SetBiClockNotifyEnabled(IATDeviceRawSIO *, bool) override {}
	void SetExternalClock(IATDeviceRawSIO *, uint32, uint32) override {}

	FakeSIO *mSIO{new FakeSIO};
};

ATDeviceSIOCommand Command(std::uint8_t device, std::uint8_t op, std::uint16_t aux) {
	ATDeviceSIOCommand command{};
	command.mDevice = device;
	command.mCommand = op;
	command.mAUX[0] = static_cast<std::uint8_t>(aux);
	command.mAUX[1] = static_cast<std::uint8_t>(aux >> 8);
	command.mbStandardRate = true;
	return command;
}

std::vector<std::uint8_t> ReadFixture(const char *path) {
	std::ifstream input(path);
	Check(input.good(), "native fixture is readable");
	const std::string json((std::istreambuf_iterator<char>(input)), {});
	// These are literal fixture vectors: the test must not grow an alternate
	// Q/W/P/R model.  The production frontend is the response authority.
	Check(json.find("\"id\":\"q-idle\"") != std::string::npos, "q fixture exists");
	Check(json.find("\"frame\":\"7f 51 00 00 d0\"") != std::string::npos, "q command checksum fixture exists");
	Check(json.find("\"id\":\"w-core-dispatch\"") != std::string::npos, "core W fixture exists");
	Check(json.find("\"payload\":\"01 02 06 00 09 00\"") != std::string::npos, "core W payload fixture exists");
	Check(json.find("\"id\":\"r-wrong-length-retains\"") != std::string::npos, "wrong-R fixture exists");
	return {0x01, 0x02, 0x06, 0x00, 0x09, 0x00};
}

void ExpectStatus(const FakeSIO& sio, const std::vector<std::uint8_t>& bytes, const char *message) {
	if (sio.transmitted != bytes)
		Fail(message);
}

void ExpectEvents(const FakeSIO& sio, std::initializer_list<const char *> expected, const char *message) {
	if (sio.events.size() != expected.size())
		Fail(message);
	std::size_t index = 0;
	for (const char *event : expected) {
		if (sio.events[index++] != event)
			Fail(message);
	}
}

} // namespace

int main(int argc, char **argv) {
	Check(argc == 2, "fixture path argument required");
	const auto request = ReadFixture(argv[1]);
	const auto requestChecksum = fujinet::io::AtariSioNioFrontend::checksum(request.data(), request.size());
	Check(requestChecksum == 0x12, "fixture data checksum matches canonical vector");

	const auto temp = std::filesystem::temp_directory_path() / "altirra-fujinet-nio-sio-test";
	std::filesystem::remove_all(temp);
	std::filesystem::create_directories(temp);
	ATPropertySet settings;
	const auto config = temp.string();
	settings.SetString("configdir", VDTextAToW(config.c_str()).c_str());
	// Exercise the same opt-in real-service composition used by the public
	// cc65 acceptance guest. This still reaches the production endpoint via
	// the IATDeviceSIO callbacks below; it is not a protocol substitute.
	settings.SetBool("publicacceptancefixture", true);

	IATDevice *device = nullptr;
	ATCreateDeviceFujiNetNIO(settings, &device);
	Check(device != nullptr, "device factory returned embedded device");
	auto *sioDevice = static_cast<IATDeviceSIO *>(device->AsInterface(IATDeviceSIO::kTypeID));
	Check(sioDevice != nullptr, "embedded device exposes IATDeviceSIO");
	FakeSIOManager manager;
	sioDevice->InitSIO(&manager);
	FakeSIO& sio = *manager.mSIO;

	// Altirra validates the command checksum before this callback; reconstruct
	// and verify the exact fifth byte the adapter gives the production endpoint.
	const auto q = Command(0x7f, 'Q', 0);
	const std::array<std::uint8_t, 4> qPrefix{q.mDevice, q.mCommand, q.mAUX[0], q.mAUX[1]};
	Check(fujinet::io::AtariSioNioFrontend::checksum(qPrefix.data(), qPrefix.size()) == 0xd0,
		"Q command checksum matches canonical fixture");
	Check(sioDevice->OnSerialBeginCommand(q) == IATDeviceSIO::kCmdResponse_Start, "Q starts");
	ExpectStatus(sio, {'A', 'C', 1, 0, 2, 0, 0x03}, "Q emits exact native reply");
	ExpectEvents(sio, {"begin", "delay", "A", "C", "data", "end"}, "Q scheduling order is exact");

	// $71 is not a native alias and must never claim the SIO transaction.
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x71, 'Q', 0)) == IATDeviceSIO::kCmdResponse_NotHandled,
		"legacy $71 is excluded");
	Check(sio.transmitted.empty(), "legacy $71 emits nothing");
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'X', 0)) == IATDeviceSIO::kCmdResponse_Start,
		"unsupported native command is claimed for canonical rejection");
	ExpectStatus(sio, {'N'}, "unsupported native command emits NAK");

	// A canonical W receives exactly AUX bytes plus checksum.  The completion
	// goes through the embedded core route and production frontend once.
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'W', request.size())) == IATDeviceSIO::kCmdResponse_Start,
		"W starts");
	ExpectStatus(sio, {'A'}, "W initial ACK is exact");
	ExpectEvents(sio, {"begin", "delay", "A", "receive", "fence"}, "W receive scheduling order is exact");
	Check(sio.receiveId == 1 && sio.fenceId == 2 &&
		sio.receiveLength == request.size() + 1 && !sio.receiveAutoProtocol,
		"W receives literal AUX bytes plus checksum");
	std::vector<std::uint8_t> framedRequest = request;
	framedRequest.push_back(requestChecksum);
	sioDevice->OnSerialReceiveComplete(sio.receiveId, framedRequest.data(), framedRequest.size(), true);
	sioDevice->OnSerialFence(sio.fenceId);
	ExpectStatus(sio, {'A', 'A', 'C'}, "W completion emits exact ACK COMPLETE");
	ExpectEvents(sio, {"begin", "delay", "A", "receive", "fence", "delay", "A", "C", "end"},
		"W completion scheduling order is exact");

	// P derives its exact ready length from the production response, then a
	// wrong R must leave that response available for the terminal R.
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'P', 0)) == IATDeviceSIO::kCmdResponse_Start, "P starts");
	Check(sio.transmitted.size() == 6 && sio.transmitted[0] == 'A' && sio.transmitted[1] == 'C' &&
		sio.transmitted[2] == 2, "P reports ready through production endpoint");
	const std::uint16_t responseLength = static_cast<std::uint16_t>(sio.transmitted[3]) |
		(static_cast<std::uint16_t>(sio.transmitted[4]) << 8);
	Check(responseLength != 0 && sio.transmitted.back() ==
		fujinet::io::AtariSioNioFrontend::checksum(sio.transmitted.data() + 2, 3),
		"P payload checksum and response length are exact");

	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'R', responseLength - 1)) == IATDeviceSIO::kCmdResponse_Start,
		"wrong R reaches adapter");
	ExpectStatus(sio, {'N'}, "wrong R is rejected without consuming reply");
	ExpectEvents(sio, {"begin", "delay", "N", "end"}, "wrong R scheduling order is exact");
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'P', 0)) == IATDeviceSIO::kCmdResponse_Start, "P after wrong R starts");
	Check(sio.transmitted.size() == 6 && sio.transmitted[2] == 2, "wrong R retains ready response");

	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'R', responseLength)) == IATDeviceSIO::kCmdResponse_Start,
		"terminal R starts");
	Check(sio.transmitted.size() == responseLength + 3 && sio.transmitted[0] == 'A' &&
		sio.transmitted[1] == 'C' && sio.transmitted.back() ==
		fujinet::io::AtariSioNioFrontend::checksum(sio.transmitted.data() + 2, responseLength),
		"terminal R keeps response storage through scheduled exact payload");
	ExpectEvents(sio, {"begin", "delay", "A", "C", "data", "end"}, "terminal R scheduling order is exact");

	// The real data callback passes a malformed checksum to the production
	// endpoint, which rejects it.  The next W is accepted directly: there is
	// no stale transaction or adapter-side replay.
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'W', request.size())) == IATDeviceSIO::kCmdResponse_Start,
		"W before bad data starts");
	framedRequest.back() ^= 0xff;
	sioDevice->OnSerialReceiveComplete(sio.receiveId, framedRequest.data(), framedRequest.size(), true);
	sioDevice->OnSerialFence(sio.fenceId);
	ExpectStatus(sio, {'A', 'N'}, "bad W checksum is rejected by production endpoint");
	ExpectEvents(sio, {"begin", "delay", "A", "receive", "fence", "N", "end"},
		"bad W rejection scheduling order is exact");
	framedRequest.back() ^= 0xff;
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'W', request.size())) == IATDeviceSIO::kCmdResponse_Start,
		"new W after rejection starts without recovery");
	sioDevice->OnSerialReceiveComplete(sio.receiveId, framedRequest.data(), framedRequest.size(), true);
	sioDevice->OnSerialFence(sio.fenceId);
	ExpectStatus(sio, {'A', 'A', 'C'}, "recovered W completion remains exact");

	// A cold reset is the only explicit virtual event that discards a ready
	// transaction; P then returns canonical idle without an unsafe R.
	device->ColdReset();
	sio.Clear();
	Check(sioDevice->OnSerialBeginCommand(Command(0x7f, 'P', 0)) == IATDeviceSIO::kCmdResponse_Start, "P after reset starts");
	ExpectStatus(sio, {'A', 'C', 0, 0, 0, 0}, "reset returns canonical idle P reply");

	device->Shutdown();
	device->Release();
	std::filesystem::remove_all(temp);
	std::puts("Altirra embedded FujiNet-NIO SIO adapter test passed");
	return 0;
}
