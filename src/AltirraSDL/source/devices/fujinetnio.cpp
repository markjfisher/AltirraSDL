//	AltirraSDL - embedded FujiNet-NIO SIO device POC
//
//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

#include <stdafx.h>
#include <at/atcore/deviceimpl.h>
#include <at/atcore/devicesioimpl.h>
#include <at/atcore/propertyset.h>

#include "fujinet/build/profile.h"
#include "fujinet/config/fuji_config.h"
#include "fujinet/core/bootstrap.h"
#include "fujinet/core/core.h"
#include "fujinet/core/device_init.h"
#include "fujinet/core/logging.h"
#include "fujinet/fs/boot_mount.h"
#include "fujinet/io/atari_sio_nio_frontend.h"
#include "fujinet/io/devices/disk_device.h"
#include "fujinet/io/devices/fuji_device.h"
#include "fujinet/io/devices/virtual_device.h"
#include "fujinet/io/protocol/wire_device_ids.h"
#include "fujinet/platform/fuji_device_factory.h"
#include "fujinet/platform/posix/fs_factory.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <vector>

namespace {

static constexpr const char *kTag = "altirra-fn";

// The virtual projection needs one deterministic packet response, but it must
// still traverse the embedded FujinetCore routing manager.  This route is
// opt-in through the test-only projectionfixture setting; it is not an SIO
// peer and it does not participate in normal embedded-device operation.
class CoreDispatchProjectionFixture final : public fujinet::io::VirtualDevice {
public:
	explicit CoreDispatchProjectionFixture(std::uint32_t& dispatches)
		: mDispatches(dispatches) {}

	fujinet::io::IOResponse handle(const fujinet::io::IORequest& request) override {
		const bool expected = request.deviceId == 1 && request.command == 2 &&
			request.params.empty() && request.payload.empty();
		if (!expected)
			return fujinet::io::make_base_response(
				request, fujinet::io::StatusCode::InvalidRequest);

		++mDispatches;
		std::fprintf(stderr,
			"[fujinetnio-projection] core dispatch fixture route invocation %u\n",
			mDispatches);
		auto response = fujinet::io::make_success_response(request);
		response.payload = {0xaa, 0x55};
		return response;
	}

private:
	std::uint32_t& mDispatches;
};

class ATDeviceFujiNetNIO final : public ATDevice, public ATDeviceSIO {
public:
	ATDeviceFujiNetNIO();
	~ATDeviceFujiNetNIO();

	void *AsInterface(uint32 id) override;
	void GetDeviceInfo(ATDeviceInfo& info) override;
	void GetSettingsBlurb(VDStringW& buf) override;
	void GetSettings(ATPropertySet& settings) override;
	bool SetSettings(const ATPropertySet& settings) override;
	void Shutdown() override;
	void ColdReset() override;

	void InitSIO(IATDeviceSIOManager *siomgr) override;
	CmdResponse OnSerialBeginCommand(const ATDeviceSIOCommand& cmd) override;
	void OnSerialAbortCommand() override;
	void OnSerialReceiveComplete(uint32 id, const void *data, uint32 len, bool checksumOK) override;
	void OnSerialFence(uint32 id) override;
	CmdResponse OnSerialAccelCommand(const ATDeviceSIORequest& request) override;

private:
	static constexpr uint32 kReceiveIdWrite = 1;
	static constexpr uint32 kFenceIdProcessWrite = 2;
	static constexpr std::uint8_t kNioSioDevice = 0x7F;
	static constexpr uint32 kCommandAckDelayCycles = 1800;

	bool InitCore();
	CmdResponse BeginCommand(const ATDeviceSIOCommand& cmd);
	void ProcessPendingWrite();
	void SendResult(const fujinet::io::AtariSioNioFrontend::Result& result);
	void ClearPendingWrite();

	IATDeviceSIOManager *mpSIOMgr = nullptr;
	vdrefptr<IATDeviceSIOInterface> mpSIOInterface;
	fujinet::core::FujinetCore mCore;
	std::unique_ptr<fujinet::io::AtariSioNioFrontend> mNio;
	VDStringW mConfigDir{L"./fujinet-data"};
	bool mProjectionFixture = false;
	std::uint32_t mProjectionDispatches = 0;
	bool mbCoreReady = false;
	std::size_t mExpectedWriteBytes = 0;
	std::vector<std::uint8_t> mPendingWrite;
	// Result bytes are retained until Altirra's queued SIO send has drained.
	std::vector<std::uint8_t> mScheduledReply;
};

} // namespace

void ATCreateDeviceFujiNetNIO(const ATPropertySet& pset, IATDevice **dev) {
	vdrefptr<ATDeviceFujiNetNIO> p(new ATDeviceFujiNetNIO);
	p->SetSettings(pset);
	*dev = p.release();
}

extern const ATDeviceDefinition g_ATDeviceDefFujiNetNIO = {
	"fujinetnio",
	"fujinetnio",
	L"FujiNet-NIO",
	ATCreateDeviceFujiNetNIO
};

ATDeviceFujiNetNIO::ATDeviceFujiNetNIO() {
	SetSaveStateAgnostic();
}

ATDeviceFujiNetNIO::~ATDeviceFujiNetNIO() {
	Shutdown();
}

void *ATDeviceFujiNetNIO::AsInterface(uint32 id) {
	switch(id) {
		case IATDeviceSIO::kTypeID:
			return static_cast<IATDeviceSIO *>(this);

		default:
			return ATDevice::AsInterface(id);
	}
}

void ATDeviceFujiNetNIO::GetDeviceInfo(ATDeviceInfo& info) {
	info.mpDef = &g_ATDeviceDefFujiNetNIO;
}

void ATDeviceFujiNetNIO::GetSettingsBlurb(VDStringW& buf) {
	buf += mConfigDir;
}

void ATDeviceFujiNetNIO::GetSettings(ATPropertySet& settings) {
	settings.SetString("configdir", mConfigDir.c_str());
	if (mProjectionFixture)
		settings.SetBool("projectionfixture", true);
}

bool ATDeviceFujiNetNIO::SetSettings(const ATPropertySet& settings) {
	const VDStringW newConfigDir(settings.GetString("configdir", L"./fujinet-data"));
	const bool newProjectionFixture = settings.GetBool("projectionfixture", false);
	if (mbCoreReady &&
		(newConfigDir != mConfigDir || newProjectionFixture != mProjectionFixture)) {
		FN_LOGW(kTag, "embedded FujiNet-NIO settings changes take effect after removing and re-adding the device");
	}
	mConfigDir = newConfigDir;
	mProjectionFixture = newProjectionFixture;
	return true;
}

void ATDeviceFujiNetNIO::Shutdown() {
	ClearPendingWrite();
	mNio.reset();
	mpSIOInterface = nullptr;
	mpSIOMgr = nullptr;
}

void ATDeviceFujiNetNIO::ColdReset() {
	ClearPendingWrite();
	if (mNio)
		mNio->reset();
}

void ATDeviceFujiNetNIO::InitSIO(IATDeviceSIOManager *siomgr) {
	mpSIOMgr = siomgr;
	mpSIOInterface = mpSIOMgr->AddDevice(this);
	if (!mbCoreReady) {
		mbCoreReady = InitCore();
	}
}

IATDeviceSIO::CmdResponse ATDeviceFujiNetNIO::OnSerialBeginCommand(const ATDeviceSIOCommand& cmd) {
	return BeginCommand(cmd);
}

void ATDeviceFujiNetNIO::OnSerialAbortCommand() {
	ClearPendingWrite();
	if (mNio)
		mNio->abort_write();
}

void ATDeviceFujiNetNIO::OnSerialReceiveComplete(uint32 id, const void *data, uint32 len, bool checksumOK) {
	if (id != kReceiveIdWrite)
		return;

	mPendingWrite.resize(len);
	if (data && len)
		std::memcpy(mPendingWrite.data(), data, len);
	// The production endpoint verifies the literal data checksum itself.  A
	// malformed Altirra receive is represented as a mismatching terminal byte
	// instead of a second adapter-local protocol decision.
	if (!checksumOK && !mPendingWrite.empty())
		mPendingWrite.back() ^= 0xff;
}

void ATDeviceFujiNetNIO::OnSerialFence(uint32 id) {
	if (id == kFenceIdProcessWrite)
		ProcessPendingWrite();
}

IATDeviceSIO::CmdResponse ATDeviceFujiNetNIO::OnSerialAccelCommand(const ATDeviceSIORequest& request) {
	(void)request;
	return kCmdResponse_BypassAccel;
}

bool ATDeviceFujiNetNIO::InitCore() {
	using namespace fujinet::io::protocol;

	const VDStringA configDir = VDTextWToU8(mConfigDir);
	std::error_code ec;
	std::filesystem::create_directories(configDir.c_str(), ec);
	if (ec) {
		FN_LOGE(kTag, "failed to create config directory '%s': %s", configDir.c_str(), ec.message().c_str());
		return false;
	}

	auto hostFs = fujinet::platform::posix::create_host_filesystem(configDir.c_str());
	if (!hostFs || !mCore.storageManager().registerFileSystem(std::move(hostFs))) {
		FN_LOGE(kTag, "failed to register host filesystem");
		return false;
	}

	auto tnfsFs = fujinet::platform::posix::create_tnfs_filesystem();
	if (!tnfsFs || !mCore.storageManager().registerFileSystem(std::move(tnfsFs))) {
		FN_LOGE(kTag, "failed to register TNFS filesystem");
		return false;
	}

	auto httpFs = fujinet::platform::posix::create_http_filesystem();
	if (!httpFs || !mCore.storageManager().registerFileSystem(std::move(httpFs))) {
		FN_LOGE(kTag, "failed to register HTTP filesystem");
		return false;
	}

	fujinet::platform::FujiDeviceHooks hooks{
		.onReset = [this] {
			FN_LOGI(kTag, "FujiNet-NIO reset requested inside AltirraSDL POC");
			ColdReset();
		}
	};

	fujinet::build::BuildProfile profile{
		.machine = fujinet::build::Machine::Atari8Bit,
		.primaryTransport = fujinet::build::TransportKind::None,
		.primaryChannel = fujinet::build::ChannelKind::SioGpio,
		.name = "AltirraSDL embedded Atari SIO ingress",
		.hw = {},
	};

	auto fuji = fujinet::platform::create_fuji_device(mCore, profile, hooks);
	auto *fujiConcrete = dynamic_cast<fujinet::io::FujiDevice *>(fuji.get());
	if (!fuji || !fujiConcrete) {
		FN_LOGE(kTag, "failed to create FujiDevice");
		return false;
	}

	const auto fujiDeviceId = to_device_id(WireDeviceId::FujiNet);
	if (!mCore.deviceManager().registerDevice(fujiDeviceId, std::move(fuji))) {
		FN_LOGE(kTag, "failed to register FujiDevice");
		return false;
	}

	fujiConcrete->start();
	const auto& config = fujiConcrete->config();

	fujinet::core::register_file_device(mCore);
	fujinet::core::register_clock_device(mCore, fujiConcrete->config_store());
	fujinet::core::register_disk_device(mCore);

	const auto diskDeviceId = to_device_id(WireDeviceId::DiskService);
	auto *diskDev = dynamic_cast<fujinet::io::DiskDevice *>(mCore.deviceManager().getDevice(diskDeviceId));
	if (diskDev) {
		constexpr std::size_t activeBootDiskUnit = 0;
		if (config.boot.mode == fujinet::config::BootMode::Config)
			diskDev->configure_boot_mount(config.boot.configUri, config.boot.readOnly);

		const auto restoredRuntimeSlots = diskDev->restore_runtime_mounts();
		const bool bootUnitRestored =
			std::find(restoredRuntimeSlots.begin(), restoredRuntimeSlots.end(),
				activeBootDiskUnit) != restoredRuntimeSlots.end();
		const std::size_t bootApplied = bootUnitRestored ? 0 :
			fujinet::apply_boot_mount(
				diskDev->disk_service(),
				mCore.storageManager(),
				config.boot,
				activeBootDiskUnit);
		FN_LOGI(kTag, "restored %zu runtime mount(s), applied %zu boot mount",
			restoredRuntimeSlots.size(), bootApplied);
	}

	fujinet::core::register_network_device(mCore, config.translation);
	fujinet::core::register_modem_device(mCore);
	if (mProjectionFixture && !mCore.deviceManager().registerDevice(
		1, std::make_unique<CoreDispatchProjectionFixture>(mProjectionDispatches))) {
		FN_LOGE(kTag, "failed to register embedded core-dispatch projection route");
		return false;
	}
	mNio = std::make_unique<fujinet::io::AtariSioNioFrontend>(mCore.routingManager());
	return true;
}

IATDeviceSIO::CmdResponse ATDeviceFujiNetNIO::BeginCommand(const ATDeviceSIOCommand& cmd) {
	if (!mbCoreReady || !mpSIOInterface || !mNio)
		return kCmdResponse_NotHandled;

	// This device owns only packet-native Atari NIO.  Disk devices retain
	// D1:-D8:, and the historic $71 stream endpoint is intentionally absent.
	if (cmd.mDevice != kNioSioDevice)
		return kCmdResponse_NotHandled;

	std::array<std::uint8_t, fujinet::io::AtariSioNioFrontend::command_bytes> frame{
		cmd.mDevice, cmd.mCommand, cmd.mAUX[0], cmd.mAUX[1], 0};
	frame.back() = fujinet::io::AtariSioNioFrontend::checksum(frame.data(), frame.size() - 1);
	const auto result = mNio->begin(frame);

	mpSIOInterface->BeginCommand();
	mpSIOInterface->Delay(kCommandAckDelayCycles);
	mScheduledReply = result.bytes;
	if (!result.accepted || mScheduledReply.empty() || mScheduledReply.front() != 'A') {
		mpSIOInterface->SendNAK();
		mpSIOInterface->EndCommand();
		return kCmdResponse_Start;
	}

	mpSIOInterface->SendACK();
	if (result.expected_write_bytes != 0) {
		mExpectedWriteBytes = result.expected_write_bytes;
		// Receive the literal data frame, including its checksum.  The endpoint
		// remains the sole owner of native W validation and state transition.
		mpSIOInterface->ReceiveData(kReceiveIdWrite,
			static_cast<uint32>(mExpectedWriteBytes + 1), false);
		mpSIOInterface->InsertFence(kFenceIdProcessWrite);
		return kCmdResponse_Start;
	}

	SendResult(result);
	return kCmdResponse_Start;
}

void ATDeviceFujiNetNIO::ClearPendingWrite() {
	mExpectedWriteBytes = 0;
	mPendingWrite.clear();
}

void ATDeviceFujiNetNIO::ProcessPendingWrite() {
	if (!mNio || mPendingWrite.size() != mExpectedWriteBytes + 1) {
		mpSIOInterface->SendNAK();
		mpSIOInterface->EndCommand();
		ClearPendingWrite();
		return;
	}

	const auto result = mNio->complete_write(mPendingWrite.data(), mPendingWrite.size());
	ClearPendingWrite();
	mScheduledReply = result.bytes;
	if (!result.accepted || mScheduledReply.empty() || mScheduledReply.front() != 'A') {
		mpSIOInterface->SendNAK();
		mpSIOInterface->EndCommand();
		return;
	}
	// The frontend's first ACK accepted W's command frame.  The physical data
	// phase still needs its own ACK before COMPLETE.  Altirra's custom-device
	// implementation enforces this 850us post-data guard as 1530 scheduler
	// cycles; without it SIOV can receive a garbled synchronous ACK.
	mpSIOInterface->Delay(1530);
	mpSIOInterface->SendACK();
	SendResult(result);
}

void ATDeviceFujiNetNIO::SendResult(const fujinet::io::AtariSioNioFrontend::Result& result) {
	(void)result;
	// ACK was emitted before this method.  The remaining frontend bytes must
	// be C + payload + checksum; use Altirra's scheduler for timing and wire
	// transmission while retaining mScheduledReply until its queue drains.
	if (mScheduledReply.size() < 2 || mScheduledReply[0] != 'A' || mScheduledReply[1] != 'C') {
		mpSIOInterface->SendNAK();
		mpSIOInterface->EndCommand();
		return;
	}
	if (mScheduledReply.size() == 2) {
		mpSIOInterface->SendComplete();
		mpSIOInterface->EndCommand();
		return;
	}

	const auto payload_bytes = mScheduledReply.size() - 3;
	const auto checksum = fujinet::io::AtariSioNioFrontend::checksum(
		mScheduledReply.data() + 2, payload_bytes);
	if (checksum != mScheduledReply.back()) {
		mpSIOInterface->SendNAK();
		mpSIOInterface->EndCommand();
		return;
	}

	mpSIOInterface->SendComplete();
	if (payload_bytes)
		mpSIOInterface->SendData(mScheduledReply.data() + 2, static_cast<uint32>(payload_bytes), true);
	mpSIOInterface->EndCommand();
}
