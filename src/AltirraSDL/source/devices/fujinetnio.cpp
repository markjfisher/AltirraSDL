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
#include "fujinet/fs/mount_applier.h"
#include "fujinet/io/core/channel.h"
#include "fujinet/io/devices/disk_device.h"
#include "fujinet/io/devices/fuji_device.h"
#include "fujinet/io/protocol/wire_device_ids.h"
#include "fujinet/platform/fuji_device_factory.h"
#include "fujinet/platform/posix/fs_factory.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {

static constexpr const char *kTag = "altirra-fn";

class AltirraFujiBusChannel final : public fujinet::io::Channel {
public:
	bool available() override {
		std::lock_guard<std::mutex> lock(mMutex);
		return !mRx.empty();
	}

	std::size_t read(std::uint8_t *buffer, std::size_t maxLen) override {
		std::lock_guard<std::mutex> lock(mMutex);
		if (!buffer || maxLen == 0 || mRx.empty())
			return 0;

		const std::size_t n = std::min(maxLen, mRx.size());
		std::copy_n(mRx.begin(), n, buffer);
		mRx.erase(mRx.begin(), mRx.begin() + static_cast<std::ptrdiff_t>(n));
		return n;
	}

	void write(const std::uint8_t *buffer, std::size_t len) override {
		std::lock_guard<std::mutex> lock(mMutex);
		if (!buffer || len == 0)
			return;

		mTx.insert(mTx.end(), buffer, buffer + len);
	}

	void PushRequest(const void *data, std::size_t len) {
		std::lock_guard<std::mutex> lock(mMutex);
		const auto *bytes = static_cast<const std::uint8_t *>(data);
		if (bytes && len)
			mRx.insert(mRx.end(), bytes, bytes + len);
	}

	std::vector<std::uint8_t> TakeResponse(std::size_t len) {
		std::lock_guard<std::mutex> lock(mMutex);
		std::vector<std::uint8_t> out(len);
		const std::size_t n = std::min(len, mTx.size());
		if (n) {
			std::copy_n(mTx.begin(), n, out.begin());
			mTx.erase(mTx.begin(), mTx.begin() + static_cast<std::ptrdiff_t>(n));
		}
		return out;
	}

	bool HasResponse() const {
		std::lock_guard<std::mutex> lock(mMutex);
		return !mTx.empty();
	}

	bool HasRequest() const {
		std::lock_guard<std::mutex> lock(mMutex);
		return !mRx.empty();
	}

	void ClearPendingWrite() {
		std::lock_guard<std::mutex> lock(mMutex);
		mRx.clear();
	}

private:
	mutable std::mutex mMutex;
	std::vector<std::uint8_t> mRx;
	std::vector<std::uint8_t> mTx;
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

	void InitSIO(IATDeviceSIOManager *siomgr) override;
	CmdResponse OnSerialBeginCommand(const ATDeviceSIOCommand& cmd) override;
	void OnSerialAbortCommand() override;
	void OnSerialReceiveComplete(uint32 id, const void *data, uint32 len, bool checksumOK) override;
	void OnSerialFence(uint32 id) override;
	CmdResponse OnSerialAccelCommand(const ATDeviceSIORequest& request) override;

private:
	static constexpr uint32 kReceiveIdWrite = 1;
	static constexpr uint32 kFenceIdProcessWrite = 2;
	static constexpr uint32 kFenceIdSendRead = 3;
	static constexpr std::uint8_t kNioSioDevice = 0x7F;
	static constexpr std::uint8_t kNetSioNetworkDevice = 0x71;
	static constexpr std::uint8_t kWriteCommand = 'W';
	static constexpr std::uint8_t kReadCommand = 'R';
	static constexpr uint32 kDefaultReadSize = 768;
	static constexpr uint32 kReadRetryDelayCycles = 1800;
	static constexpr uint32 kReadMaxRetries = 10000;

	static bool IsNioSioDevice(std::uint8_t device) {
		return device == kNioSioDevice || device == kNetSioNetworkDevice;
	}

	static uint32 AuxLength(const ATDeviceSIOCommand& cmd) {
		return static_cast<uint32>(cmd.mAUX[0]) | (static_cast<uint32>(cmd.mAUX[1]) << 8);
	}

	static uint32 TransferLength(const ATDeviceSIOCommand& cmd, uint32 explicitLength) {
		if (explicitLength)
			return explicitLength;

		const uint32 aux = AuxLength(cmd);
		return aux ? aux : kDefaultReadSize;
	}

	bool InitCore();
	void StartWorker();
	void StopWorker();
	void WorkerLoop();
	CmdResponse BeginWriteCommand(const ATDeviceSIOCommand& cmd, uint32 len);
	CmdResponse BeginReadCommand(const ATDeviceSIOCommand& cmd, uint32 len);
	CmdResponse BeginCommand(const ATDeviceSIOCommand& cmd, uint32 explicitLength);
	void ProcessPendingWrite();
	void SendReadResponseOrRetry();

	IATDeviceSIOManager *mpSIOMgr = nullptr;
	vdrefptr<IATDeviceSIOInterface> mpSIOInterface;
	fujinet::core::FujinetCore mCore;
	AltirraFujiBusChannel mChannel;
	VDStringW mConfigDir{L"./fujinet-data"};
	bool mbCoreReady = false;
	bool mbPendingWrite = false;
	bool mbPendingWriteChecksumOK = false;
	uint32 mPendingReadSize = 0;
	uint32 mPendingReadRetries = 0;
	std::vector<std::uint8_t> mPendingWrite;
	std::mutex mWorkerMutex;
	std::condition_variable mWorkerCv;
	std::thread mWorker;
	bool mbWorkerStop = false;
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
	StopWorker();
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
}

bool ATDeviceFujiNetNIO::SetSettings(const ATPropertySet& settings) {
	const VDStringW newConfigDir(settings.GetString("configdir", L"./fujinet-data"));
	if (mbCoreReady && newConfigDir != mConfigDir) {
		FN_LOGW(kTag, "configdir changes take effect after removing and re-adding the device");
	}
	mConfigDir = newConfigDir;
	return true;
}

void ATDeviceFujiNetNIO::Shutdown() {
	StopWorker();
	mpSIOInterface = nullptr;
	mpSIOMgr = nullptr;
}

void ATDeviceFujiNetNIO::InitSIO(IATDeviceSIOManager *siomgr) {
	mpSIOMgr = siomgr;
	mpSIOInterface = mpSIOMgr->AddDevice(this);
	if (!mbCoreReady) {
		mbCoreReady = InitCore();
		if (mbCoreReady)
			StartWorker();
	}
}

IATDeviceSIO::CmdResponse ATDeviceFujiNetNIO::OnSerialBeginCommand(const ATDeviceSIOCommand& cmd) {
	return BeginCommand(cmd, 0);
}

void ATDeviceFujiNetNIO::OnSerialAbortCommand() {
	mbPendingWrite = false;
	mbPendingWriteChecksumOK = false;
	mPendingWrite.clear();
	mChannel.ClearPendingWrite();
}

void ATDeviceFujiNetNIO::OnSerialReceiveComplete(uint32 id, const void *data, uint32 len, bool checksumOK) {
	if (id != kReceiveIdWrite)
		return;

	mbPendingWrite = true;
	mbPendingWriteChecksumOK = checksumOK;
	mPendingWrite.resize(len);
	if (data && len)
		std::memcpy(mPendingWrite.data(), data, len);
}

void ATDeviceFujiNetNIO::OnSerialFence(uint32 id) {
	if (id == kFenceIdProcessWrite)
		ProcessPendingWrite();
	else if (id == kFenceIdSendRead)
		SendReadResponseOrRetry();
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
		.onReset = [] {
			FN_LOGI(kTag, "FujiNet-NIO reset requested inside AltirraSDL POC");
		}
	};

	fujinet::build::BuildProfile profile{
		fujinet::build::Machine::Atari8Bit,
		fujinet::build::TransportKind::FujiBus,
		fujinet::build::ChannelKind::SioGpio,
		"AltirraSDL embedded FujiNet-NIO",
		{}
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
		const std::size_t applied = fujinet::apply_config_mounts(
			diskDev->disk_service(),
			mCore.storageManager(),
			config.mounts);
		FN_LOGI(kTag, "applied %zu config mount(s)", applied);
	}

	fujinet::core::register_network_device(mCore);
	fujinet::core::register_modem_device(mCore);
	fujinet::core::setup_transports(mCore, mChannel, profile, &config);
	return true;
}

void ATDeviceFujiNetNIO::StartWorker() {
	std::lock_guard<std::mutex> lock(mWorkerMutex);
	if (mWorker.joinable())
		return;

	mbWorkerStop = false;
	mWorker = std::thread([this] {
		WorkerLoop();
	});
}

void ATDeviceFujiNetNIO::StopWorker() {
	{
		std::lock_guard<std::mutex> lock(mWorkerMutex);
		mbWorkerStop = true;
	}
	mWorkerCv.notify_all();

	if (mWorker.joinable())
		mWorker.join();
}

void ATDeviceFujiNetNIO::WorkerLoop() {
	for(;;) {
		{
			std::unique_lock<std::mutex> lock(mWorkerMutex);
			mWorkerCv.wait_for(lock, std::chrono::milliseconds(1), [this] {
				return mbWorkerStop || mChannel.HasRequest();
			});

			if (mbWorkerStop)
				break;
		}

		mCore.tick();
	}
}

IATDeviceSIO::CmdResponse ATDeviceFujiNetNIO::BeginWriteCommand(const ATDeviceSIOCommand& cmd, uint32 len) {
	(void)cmd;

	if (!len)
		return kCmdResponse_Fail_NAK;

	mpSIOInterface->BeginCommand();
	mpSIOInterface->SendACK();
	mpSIOInterface->ReceiveData(kReceiveIdWrite, len, true);
	mpSIOInterface->InsertFence(kFenceIdProcessWrite);
	return kCmdResponse_Start;
}

IATDeviceSIO::CmdResponse ATDeviceFujiNetNIO::BeginReadCommand(const ATDeviceSIOCommand& cmd, uint32 len) {
	(void)cmd;

	const uint32 transferSize = len ? len : kDefaultReadSize;
	mPendingReadSize = transferSize;
	mPendingReadRetries = 0;

	mpSIOInterface->BeginCommand();
	mpSIOInterface->SendACK();
	mpSIOInterface->InsertFence(kFenceIdSendRead);
	return kCmdResponse_Start;
}

IATDeviceSIO::CmdResponse ATDeviceFujiNetNIO::BeginCommand(const ATDeviceSIOCommand& cmd, uint32 explicitLength) {
	if (!mbCoreReady || !mpSIOInterface)
		return kCmdResponse_NotHandled;

	if (!IsNioSioDevice(cmd.mDevice))
		return kCmdResponse_NotHandled;

	const uint32 len = TransferLength(cmd, explicitLength);
	switch(cmd.mCommand) {
		case kWriteCommand:
			return BeginWriteCommand(cmd, len);

		case kReadCommand:
			return BeginReadCommand(cmd, len);

		default:
			return kCmdResponse_Fail_NAK;
	}
}

void ATDeviceFujiNetNIO::ProcessPendingWrite() {
	if (!mbPendingWrite) {
		mpSIOInterface->SendError();
		mpSIOInterface->EndCommand();
		return;
	}

	if (!mbPendingWriteChecksumOK) {
		FN_LOGW(kTag, "write checksum failed");
		mbPendingWrite = false;
		mbPendingWriteChecksumOK = false;
		mPendingWrite.clear();
		mpSIOInterface->SendError();
		mpSIOInterface->EndCommand();
		return;
	}

	mChannel.PushRequest(mPendingWrite.data(), mPendingWrite.size());
	mWorkerCv.notify_one();

	mbPendingWrite = false;
	mbPendingWriteChecksumOK = false;
	mPendingWrite.clear();

	mpSIOInterface->SendComplete();
	mpSIOInterface->EndCommand();
}

void ATDeviceFujiNetNIO::SendReadResponseOrRetry() {
	if (mChannel.HasResponse()) {
		auto response = mChannel.TakeResponse(mPendingReadSize);
		mpSIOInterface->SendComplete();
		mpSIOInterface->SendData(response.data(), mPendingReadSize, true);
		mpSIOInterface->EndCommand();
		mPendingReadSize = 0;
		mPendingReadRetries = 0;
		return;
	}

	if (++mPendingReadRetries > kReadMaxRetries) {
		FN_LOGW(kTag, "timed out waiting for FujiBus response");
		mpSIOInterface->SendError();
		mpSIOInterface->EndCommand();
		mPendingReadSize = 0;
		mPendingReadRetries = 0;
		return;
	}

	mWorkerCv.notify_one();
	mpSIOInterface->Delay(kReadRetryDelayCycles);
	mpSIOInterface->InsertFence(kFenceIdSendRead);
}
