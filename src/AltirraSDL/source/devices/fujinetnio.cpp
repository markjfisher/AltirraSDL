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
#include "fujinet/io/devices/network_protocol.h"
#include "fujinet/io/devices/network_protocol_registry.h"
#include "fujinet/io/core/request_handler.h"
#include "fujinet/io/devices/virtual_device.h"
#include "fujinet/io/protocol/wire_device_ids.h"
#include "fujinet/disk/disk_types.h"
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
#include <optional>
#include <vector>

#if defined(ALTIRRA_FUJINET_NIO_TEST_CLOCK)
namespace fujinet::platform {
void set_test_unix_time_seconds(std::optional<std::uint64_t> frozen);
}
#endif

namespace {

static constexpr const char *kTag = "altirra-fn";
static constexpr const char *kAcceptanceTag = "atari-public-acceptance";
static constexpr std::uint64_t kAcceptanceClockUnixSeconds = 0x0102030405060708ULL;
static constexpr const char *kAcceptanceNetworkUri = "fixture://nio-accept";
static constexpr const char *kAcceptanceNetworkWrite = "NET-WRITE-24";
static constexpr const char *kAcceptanceNetworkReply = "NET-REPLY-24";
static constexpr const char *kAcceptanceDiskPath = "/fnaccept-disk.atr";
static constexpr std::size_t kAcceptanceDiskSlot = 1; // Guest slot 2.
static constexpr std::uint16_t kAcceptanceDiskSectorBytes = 128;
static constexpr std::uint32_t kAcceptanceDiskSectors = 16;

std::uint8_t CapacityPattern(std::size_t packetOffset) {
	return static_cast<std::uint8_t>(packetOffset ^ (packetOffset >> 8U) ^ 0xa5U);
}

bool IsCapacityPayload(const std::vector<std::uint8_t>& payload,
	std::size_t packetOffset) {
	for (std::size_t index = 0; index < payload.size(); ++index) {
		if (payload[index] != CapacityPattern(packetOffset + index))
			return false;
	}
	return true;
}

// The virtual projection needs one deterministic packet response, but it must
// still traverse the embedded FujinetCore routing manager.  This route is
// opt-in through the test-only projectionfixture setting; it is not an SIO
// peer and it does not participate in normal embedded-device operation.
class CoreDispatchProjectionFixture final : public fujinet::io::VirtualDevice {
public:
	explicit CoreDispatchProjectionFixture(std::uint32_t& dispatches)
		: mDispatches(dispatches) {}

	fujinet::io::IOResponse handle(const fujinet::io::IORequest& request) override {
		const bool smallRequest = request.deviceId == 1 &&
			request.command >= 2 && request.command <= 4 &&
			request.params.empty() && request.payload.empty();
		const bool request511 = request.deviceId == 1 && request.command == 5 &&
			request.params.empty() && request.payload.size() == 505 &&
			IsCapacityPayload(request.payload, 6);
		const bool request512 = request.deviceId == 1 && request.command == 6 &&
			request.params.empty() && request.payload.size() == 506 &&
			IsCapacityPayload(request.payload, 6);
		const bool oversizeResponseRequest = request.deviceId == 1 &&
			request.command == 7 && request.params.empty() && request.payload.empty();
		if (!(smallRequest || request511 || request512 || oversizeResponseRequest))
			return fujinet::io::make_base_response(
				request, fujinet::io::StatusCode::InvalidRequest);

		++mDispatches;
		std::fprintf(stderr,
			"[fujinetnio-projection] core dispatch fixture route invocation %u\n",
			mDispatches);
		auto response = fujinet::io::make_success_response(request);
		// The reset and interrupted-R projections use distinct small responses;
		// capacity command 6 emits one full, non-repeating 512-byte raw packet.
		if (request.command == 2) {
			response.payload = {0xaa, 0x55};
		} else if (request.command == 3) {
			response.payload = {0x33, 0xcc};
		} else if (request.command == 4) {
			response.payload = {0x5a, 0xa5};
		} else if (request.command == 6) {
			response.payload.resize(505);
			for (std::size_t index = 0; index < response.payload.size(); ++index)
				response.payload[index] = CapacityPattern(7 + index);
		} else if (request.command == 7) {
			// This serializes to 513 raw FujiBus bytes. The production frontend
			// must replace it with its bounded InternalError response.
			response.payload.resize(506);
			for (std::size_t index = 0; index < response.payload.size(); ++index)
				response.payload[index] = CapacityPattern(7 + index);
		}
		return response;
	}

private:
	std::uint32_t& mDispatches;
};

// This opt-in fixture composes the ordinary production service devices for
// the public cc65 acceptance guest.  It does not handle SIO or FujiBus: the
// embedded AtariSioNioFrontend still owns every Q/W/P/R transaction and the
// normal RoutingManager still owns the real device dispatch.
struct PublicAcceptanceNetworkState {
	std::uint32_t opens{};
	std::uint32_t writes{};
	std::uint32_t closes{};
	std::vector<std::uint8_t> lastWrite;
};

class PublicAcceptanceNetworkProtocol final : public fujinet::io::INetworkProtocol {
public:
	explicit PublicAcceptanceNetworkProtocol(std::shared_ptr<PublicAcceptanceNetworkState> state)
		: mState(std::move(state)) {}

	fujinet::io::StatusCode open(const fujinet::io::NetworkOpenRequest& request) override {
		++mState->opens;
		FN_LOGI(kAcceptanceTag, "Network fixture open count=%u method=%u url=%s",
			mState->opens, static_cast<unsigned>(request.method), request.url.c_str());
		return request.method == 2 && request.url == kAcceptanceNetworkUri
			? fujinet::io::StatusCode::Ok : fujinet::io::StatusCode::InvalidRequest;
	}

	fujinet::io::StatusCode write_body(std::uint32_t offset,
		const std::uint8_t* data, std::size_t length, std::uint16_t& written) override {
		++mState->writes;
		mState->lastWrite.assign(data, data + length);
		const bool valid = offset == 0 && length == std::strlen(kAcceptanceNetworkWrite) &&
			std::equal(mState->lastWrite.begin(), mState->lastWrite.end(),
				kAcceptanceNetworkWrite);
		FN_LOGI(kAcceptanceTag, "Network fixture write count=%u bytes=%u valid=%u",
			mState->writes, static_cast<unsigned>(length), valid ? 1u : 0u);
		written = valid ? static_cast<std::uint16_t>(length) : 0;
		return valid ? fujinet::io::StatusCode::Ok : fujinet::io::StatusCode::InvalidRequest;
	}

	fujinet::io::StatusCode read_body(std::uint32_t offset, std::uint8_t* out,
		std::size_t capacity, std::uint16_t& read, bool& eof, bool& moreAvailable) override {
		const std::size_t replyLength = std::strlen(kAcceptanceNetworkReply);
		if (offset > replyLength)
			return fujinet::io::StatusCode::InvalidRequest;
		const std::size_t available = replyLength - static_cast<std::size_t>(offset);
		const std::size_t count = std::min(available, capacity);
		std::copy_n(reinterpret_cast<const std::uint8_t*>(kAcceptanceNetworkReply) + offset,
			count, out);
		read = static_cast<std::uint16_t>(count);
		eof = count == available;
		moreAvailable = !eof;
		return fujinet::io::StatusCode::Ok;
	}

	fujinet::io::StatusCode info(fujinet::io::NetworkInfo& out) override {
		out = fujinet::io::NetworkInfo{};
		return fujinet::io::StatusCode::Ok;
	}

	void poll() override {}

	void close() override {
		++mState->closes;
		FN_LOGI(kAcceptanceTag, "Network fixture close count=%u", mState->closes);
	}

private:
	std::shared_ptr<PublicAcceptanceNetworkState> mState;
};

class PublicAcceptanceRoute final : public fujinet::io::IRequestHandler {
public:
	explicit PublicAcceptanceRoute(fujinet::io::IRequestHandler& target)
		: mTarget(target) {}

	fujinet::io::IOResponse handleRequest(const fujinet::io::IORequest& request) override {
		using fujinet::io::protocol::WireDeviceId;
		using fujinet::io::protocol::to_device_id;
		if (request.deviceId == to_device_id(WireDeviceId::Clock)) {
			FN_LOGI(kAcceptanceTag, "ClockDevice routed request %u command=%u",
				++mClockDispatches, static_cast<unsigned>(request.command));
		} else if (request.deviceId == to_device_id(WireDeviceId::FileService)) {
			FN_LOGI(kAcceptanceTag, "FileDevice routed request %u command=%u",
				++mFileDispatches, static_cast<unsigned>(request.command));
		} else if (request.deviceId == to_device_id(WireDeviceId::NetworkService)) {
			FN_LOGI(kAcceptanceTag, "NetworkDevice routed request %u command=%u",
				++mNetworkDispatches, static_cast<unsigned>(request.command));
		} else if (request.deviceId == to_device_id(WireDeviceId::DiskService)) {
			FN_LOGI(kAcceptanceTag, "DiskDevice routed request %u command=%u",
				++mDiskDispatches, static_cast<unsigned>(request.command));
		}
		return mTarget.handleRequest(request);
	}

private:
	fujinet::io::IRequestHandler& mTarget;
	std::uint32_t mClockDispatches{};
	std::uint32_t mFileDispatches{};
	std::uint32_t mNetworkDispatches{};
	std::uint32_t mDiskDispatches{};
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
	bool EnablePublicAcceptanceFixture(fujinet::io::DiskDevice& disk);
	CmdResponse BeginCommand(const ATDeviceSIOCommand& cmd);
	void ProcessPendingWrite();
	void SendResult(const fujinet::io::AtariSioNioFrontend::Result& result);
	void ClearPendingWrite();

	IATDeviceSIOManager *mpSIOMgr = nullptr;
	vdrefptr<IATDeviceSIOInterface> mpSIOInterface;
	fujinet::core::FujinetCore mCore;
	std::unique_ptr<PublicAcceptanceRoute> mPublicAcceptanceRoute;
	std::unique_ptr<fujinet::io::AtariSioNioFrontend> mNio;
	VDStringW mConfigDir{L"./fujinet-data"};
	bool mProjectionFixture = false;
	bool mPublicAcceptanceFixture = false;
	// Test-only Altirra scheduling fault. It is deliberately downstream of
	// AtariSioNioFrontend: the endpoint accepts and consumes terminal R before
	// a corrupted serial checksum reaches the guest.
	bool mProjectionInterruptTerminalR = false;
	bool mProjectionTerminalRFaultInjected = false;
	std::uint8_t mActiveNioCommand = 0;
	std::uint32_t mProjectionDispatches = 0;
	bool mbCoreReady = false;
	std::size_t mExpectedWriteBytes = 0;
	std::vector<std::uint8_t> mPendingWrite;
	// Result bytes are retained until Altirra's queued SIO send has drained.
	std::vector<std::uint8_t> mScheduledReply;
	std::vector<std::uint8_t> mFaultScheduledReply;
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
	if (mPublicAcceptanceFixture)
		settings.SetBool("publicacceptancefixture", true);
	if (mProjectionInterruptTerminalR)
		settings.SetBool("projectioninterruptterminalr", true);
}

bool ATDeviceFujiNetNIO::SetSettings(const ATPropertySet& settings) {
	const VDStringW newConfigDir(settings.GetString("configdir", L"./fujinet-data"));
	const bool newProjectionFixture = settings.GetBool("projectionfixture", false);
	const bool newPublicAcceptanceFixture = settings.GetBool("publicacceptancefixture", false);
	const bool newProjectionInterruptTerminalR = settings.GetBool("projectioninterruptterminalr", false);
	if (newProjectionInterruptTerminalR && !newProjectionFixture) {
		FN_LOGE(kTag, "projectioninterruptterminalr requires projectionfixture=true");
		return false;
	}
	if (newProjectionFixture && newPublicAcceptanceFixture) {
		FN_LOGE(kTag, "projectionfixture and publicacceptancefixture are mutually exclusive");
		return false;
	}
	if (mbCoreReady &&
		(newConfigDir != mConfigDir || newProjectionFixture != mProjectionFixture ||
		 newPublicAcceptanceFixture != mPublicAcceptanceFixture ||
		 newProjectionInterruptTerminalR != mProjectionInterruptTerminalR)) {
		FN_LOGW(kTag, "embedded FujiNet-NIO settings changes take effect after removing and re-adding the device");
	}
	mConfigDir = newConfigDir;
	mProjectionFixture = newProjectionFixture;
	mPublicAcceptanceFixture = newPublicAcceptanceFixture;
	mProjectionInterruptTerminalR = newProjectionInterruptTerminalR;
	mProjectionTerminalRFaultInjected = false;
	return true;
}

void ATDeviceFujiNetNIO::Shutdown() {
	ClearPendingWrite();
	mNio.reset();
	mPublicAcceptanceRoute.reset();
#if defined(ALTIRRA_FUJINET_NIO_TEST_CLOCK)
	if (mPublicAcceptanceFixture)
		fujinet::platform::set_test_unix_time_seconds(std::nullopt);
#endif
	mpSIOInterface = nullptr;
	mpSIOMgr = nullptr;
}

void ATDeviceFujiNetNIO::ColdReset() {
	FN_LOGI(kTag, "embedded FujiNet-NIO device ColdReset");
	ClearPendingWrite();
	mProjectionTerminalRFaultInjected = false;
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

	if (mPublicAcceptanceFixture) {
#if defined(ALTIRRA_FUJINET_NIO_TEST_CLOCK)
		fujinet::platform::set_test_unix_time_seconds(kAcceptanceClockUnixSeconds);
#else
		FN_LOGE(kTag, "publicacceptancefixture requires the embedded test clock");
		return false;
#endif
	}

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
		if (mPublicAcceptanceFixture && !EnablePublicAcceptanceFixture(*diskDev))
			return false;
	} else if (mPublicAcceptanceFixture) {
		FN_LOGE(kTag, "publicacceptancefixture requires DiskDevice");
		return false;
	}

	if (mPublicAcceptanceFixture) {
		auto networkState = std::make_shared<PublicAcceptanceNetworkState>();
		fujinet::io::ProtocolRegistry registry;
		if (!registry.register_scheme("fixture", [networkState] {
				return std::make_unique<PublicAcceptanceNetworkProtocol>(networkState);
			})) {
			FN_LOGE(kTag, "failed to register public acceptance Network fixture");
			return false;
		}
		fujinet::core::register_network_device(mCore, std::move(registry), {});
	} else {
		fujinet::core::register_network_device(mCore, config.translation);
	}
	fujinet::core::register_modem_device(mCore);
	if (mProjectionFixture && !mCore.deviceManager().registerDevice(
		1, std::make_unique<CoreDispatchProjectionFixture>(mProjectionDispatches))) {
		FN_LOGE(kTag, "failed to register embedded core-dispatch projection route");
		return false;
	}
	if (mPublicAcceptanceFixture) {
		mPublicAcceptanceRoute = std::make_unique<PublicAcceptanceRoute>(mCore.routingManager());
		mNio = std::make_unique<fujinet::io::AtariSioNioFrontend>(*mPublicAcceptanceRoute);
		FN_LOGI(kAcceptanceTag,
			"embedded public API Clock/File/Network/Disk acceptance initialized time=0807060504030201");
	} else {
		mNio = std::make_unique<fujinet::io::AtariSioNioFrontend>(mCore.routingManager());
	}
	return true;
}

bool ATDeviceFujiNetNIO::EnablePublicAcceptanceFixture(fujinet::io::DiskDevice& disk) {
	const auto created = disk.disk_service().create_image(
		"host", kAcceptanceDiskPath, fujinet::disk::ImageType::Atr,
		kAcceptanceDiskSectorBytes, kAcceptanceDiskSectors, true);
	if (!created.ok()) {
		FN_LOGE(kTag, "failed to create public acceptance ATR error=%u",
			static_cast<unsigned>(created.error));
		return false;
	}
	fujinet::disk::MountOptions options{};
	options.readOnlyRequested = false;
	const auto mounted = disk.disk_service().mount(
		kAcceptanceDiskSlot, "host", kAcceptanceDiskPath, options);
	if (!mounted.ok()) {
		FN_LOGE(kTag, "failed to mount public acceptance ATR error=%u",
			static_cast<unsigned>(mounted.error));
		return false;
	}
	FN_LOGI(kAcceptanceTag, "Disk fixture mounted slot=%u path=%s sector_bytes=%u sectors=%u",
		static_cast<unsigned>(kAcceptanceDiskSlot + 1), kAcceptanceDiskPath,
		static_cast<unsigned>(kAcceptanceDiskSectorBytes),
		static_cast<unsigned>(kAcceptanceDiskSectors));
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
	mActiveNioCommand = cmd.mCommand;
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
	if (payload_bytes) {
		const bool interruptTerminalR = mProjectionInterruptTerminalR &&
			!mProjectionTerminalRFaultInjected && mActiveNioCommand == 'R';
		if (interruptTerminalR) {
			mProjectionTerminalRFaultInjected = true;
			// SendData(false) exposes the literal raw serial bytes to Altirra's
			// SIO scheduler.  Retain the corrupt delivery frame separately from
			// mScheduledReply, whose unmodified bytes remain the endpoint's
			// accepted response ownership.
			mFaultScheduledReply.assign(mScheduledReply.begin() + 2,
				mScheduledReply.begin() + 2 + payload_bytes);
			mFaultScheduledReply.push_back(static_cast<std::uint8_t>(checksum ^ 0x01));
			FN_LOGI(kTag, "projection fault: accepted terminal R checksum %02x delivered as %02x",
				checksum, mFaultScheduledReply.back());
			mpSIOInterface->SendData(mFaultScheduledReply.data(),
				static_cast<uint32>(mFaultScheduledReply.size()), false);
		} else {
			mpSIOInterface->SendData(mScheduledReply.data() + 2,
				static_cast<uint32>(payload_bytes), true);
		}
	}
	mpSIOInterface->EndCommand();
}
