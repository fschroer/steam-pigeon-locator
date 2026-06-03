extern void *__libc_init_array;

extern "C" {
#include <stdio.h>
}

#include <cstring>
#include "Communication.hpp"
#include "CubeMonitorGlobals.hpp"
#include "StRadioAdapter.hpp"
#include "RocketSettings.hpp"
#include "Format.hpp"
#include "Units.hpp"
#include "UserInteraction.hpp"
#include "version.h"
#include "RgbLed.hpp"

namespace Communication {

namespace FA = FlightArchive;
using Sample = FA::FlightSample;
using MsgType = MsgType;

Communication::Communication(DeviceUID &deviceUID, FlightManager &flight, RocketNav::Navigation &nav, Archive &archive,
		PowerManagement &power, Deployment &deploy) :
		deviceUID_(deviceUID), flight_(flight), nav_(nav), archive_(archive), power_(power), deploy_(deploy) {
}

// ============================================================================
//  Initialisation
// ============================================================================

void Communication::Init(IRadio &radio) {
	radio_ = &radio;
	radio_->SetChannel(902300000 + archive_.GetLocatorSettings().lora_channel * 200000);

	StartupMessage msg;
	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::Startup;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0;
	msg.serial_number = deviceUID_.getUID();
	std::memcpy(msg.version, GIT_VERSION, std::min(sizeof(msg.version), sizeof(GIT_VERSION)));
	msg.packet_header.crc = ComputeMessageCrc(msg);
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

void Communication::SetChannel(uint8_t channel) {
	radio_->SetChannel(902300000 + channel * 200000);
}

void Communication::SendGenericPacket(const uint8_t *data, size_t len) {
	radio_->Send(data, len);
}

// ============================================================================
//  Pre-launch / telemetry packets
// ============================================================================

void Communication::SendPreLaunchData() {
	PreLaunchData msg;
	NavSolution nav_solution = nav_.getFused();
	GpsSample gps_sample = nav_.getRawGps();
	RocketPersistentSettings rocket_settings = archive_.GetLocatorSettings();

	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::PreLaunchData;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0;

	msg.latitude = nav_solution.pos.lat_rad * RAD2DEG;
	msg.longitude = nav_solution.pos.lon_rad * RAD2DEG;
	msg.satellites = gps_sample.num_sv;
	msg.hacc = gps_sample.h_acc_m;
	msg.imu_status = nav_.imuStatus().health;
	msg.baro_status = nav_.baroStatus().health;
	msg.gps_status = nav_.gpsStatus().health;
	msg.deploy_status = DeploymentChannelContinuity();
	msg.agl = nav_solution.altitude_agl_m;
	msg.accel = nav_solution.body_accel_mps2;
	msg.gyro = nav_solution.body_rates_rps;
	msg.deploy_ch1_mode = rocket_settings.deployment_ch1_mode;
	msg.deploy_ch2_mode = rocket_settings.deployment_ch2_mode;
	msg.deploy_ch3_mode = rocket_settings.deployment_ch3_mode;
	msg.deploy_ch4_mode = rocket_settings.deployment_ch4_mode;
	msg.drogue_primary_deploy_delay = rocket_settings.drogue_primary_deploy_delay;
	msg.drogue_backup_deploy_delay = rocket_settings.drogue_backup_deploy_delay;
	msg.main_primary_deploy_altitude = rocket_settings.main_primary_deploy_altitude;
	msg.main_backup_deploy_altitude = rocket_settings.main_backup_deploy_altitude;
	std::memcpy(msg.device_name, rocket_settings.device_name, device_name_length);
	msg.battery_voltage_mvolt = power_.readBatteryMillivolts();

	msg.packet_header.crc = ComputeMessageCrc(msg);
	RgbLed(RgbColor::Blue); // Blink LoRa transmit LED for visual validation
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(PreLaunchData));
}

void Communication::SendTelemetryData() {
	TelemetryData msg;
	NavSolution nav_solution = nav_.getFused();
	GpsSample gps_sample = nav_.getRawGps();

	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::TelemetryData;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0;

	msg.latitude = nav_solution.pos.lat_rad * RAD2DEG;
	msg.longitude = nav_solution.pos.lon_rad * RAD2DEG;
	msg.satellites = gps_sample.num_sv;
	msg.hacc = gps_sample.h_acc_m;
	msg.imu_status = nav_.imuStatus().health;
	msg.baro_status = nav_.baroStatus().health;
	msg.gps_status = nav_.gpsStatus().health;
	uint8_t status = DeploymentChannelContinuity();
	msg.deployment_ch1_stats = flight_.GetDeploymentStats(1) | ((status & 0x01) << bit_shift_continuity);
	msg.deployment_ch2_stats = flight_.GetDeploymentStats(2) | ((status & 0x02) << (bit_shift_continuity - 1));
	msg.deployment_ch3_stats = flight_.GetDeploymentStats(3) | ((status & 0x04) << (bit_shift_continuity - 2));
	msg.deployment_ch4_stats = flight_.GetDeploymentStats(4) | ((status & 0x08) << (bit_shift_continuity - 3));
	msg.physical_deployment_stats = flight_.GetPhysicalDeploymentStats();
	msg.agl = nav_solution.altitude_agl_m;
	msg.accel = nav_solution.body_accel_mps2;
	msg.gyro = nav_solution.body_rates_rps;
	msg.velocity = nav_solution.vertical_speed_mps;
	msg.flight_state = flight_.GetFlightState();

	msg.packet_header.crc = ComputeMessageCrc(msg);
	RgbLed(RgbColor::Blue); // Blink LoRa transmit LED for visual validation
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(TelemetryData));
}

void Communication::SendTestCountdownMessage(uint16_t test_deploy_count) {
	DeploymentTestCountdownMessage msg { };
	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::DeploymentTest;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0;

	msg.count = test_deploy_count / SAMPLES_PER_SECOND;
	msg.packet_header.crc = ComputeMessageCrc(msg);
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(DeploymentTestCountdownMessage));
}

// ============================================================================
//  Flight profile metadata / data
// ============================================================================

void Communication::SendFlightProfileMetadata(DeviceState &device_state) {
	if (send_metadata_ && HAL_GetTick() - send_metadata_request_time_ > 50) {
		send_metadata_ = false;
		FlightMetadata msg;
		msg.packet_header.system_id = system_id;
		msg.packet_header.msg_type = MsgType::FlightMetadata;
		msg.packet_header.msg_count = 0;
		msg.packet_header.crc = 0;
		bool present = false;
		for (uint8_t i = 0; i < record_count; i++) {
			archive_.ReadEvent(i, FlightArchive::ExampleStatId::FlightTimestampS, msg.record[i].timestamp, present);
			archive_.ReadEvent(i, FlightArchive::ExampleStatId::MaxAltitudeM, msg.record[i].apogee, present);
			archive_.ReadEvent(i, FlightArchive::ExampleStatId::LandingTimestampMs, msg.record[i].flight_time, present);
		}
		msg.packet_header.crc = ComputeMessageCrc(msg);
		RgbLed(RgbColor::Blue); // Blink LoRa transmit LED for visual validation
		radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(FlightMetadata));
		flight_profile_active_ms_ = HAL_GetTick();
		// Stay in MetadataRequested so PreLaunchData is not resumed until the
		// app explicitly exits the flight profile screen (sends DisarmRequest).
	}
}

void Communication::SendFlightProfileData() {
	// Intentionally thin: transfer is driven by Process() after BeginTransfer()
	// is called from OnRadioRxDone(). Nothing to do here.
}

// ============================================================================
//  Radio callbacks
// ============================================================================

void Communication::OnRadioTxDone() {
	radio_busy_ = false;
	last_radio_tx_end_ms_ = HAL_GetTick();
}

void Communication::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo,
		DeviceState &device_state) {
	RgbLed(RgbColor::Green);

	ParsedMessage parsed { };
	if (ParseLoraFrame(payload, size, system_id, parsed) == ParseResult::Ok) {
		FlightStates flight_state = flight_.GetFlightState();
		switch (parsed.type) {

		case MsgType::LocatorCfgChgRequest: {
			RocketPersistentSettings rocket_settings { };
			std::memcpy(&rocket_settings, payload + sizeof(PacketHeader), sizeof(rocket_settings));
			archive_.SaveLocatorSettings(rocket_settings);
			break;
		}

		case MsgType::ArmRequest:
			if (device_state == DeviceState::Disarmed)
				device_state = DeviceState::Armed;
			break;

		case MsgType::DisarmRequest:
			// Only allow disarm before or after a flight
			if (flight_state == FlightStates::WaitingLaunch || flight_state == FlightStates::Landed)
				device_state = DeviceState::Disarmed;
			break;

		case MsgType::FlightMetadataRequest:
			// Accept from any non-Armed state: Disarmed (initial entry),
			// MetadataRequested (re-request before prior response was used),
			// or DataRequested (user returned to the flight list).
			if (device_state == DeviceState::Disarmed || device_state == DeviceState::MetadataRequested
					|| device_state == DeviceState::DataRequested) {
				device_state = DeviceState::MetadataRequested;
				send_metadata_ = true;
				send_metadata_request_time_ = HAL_GetTick();
				flight_profile_active_ms_ = HAL_GetTick();
			}
			break;

		case MsgType::FlightDataRequest: {
			// Accept from Disarmed, MetadataRequested, or DataRequested (retry /
			// select a different flight record while already in data mode).
			if (device_state == DeviceState::Disarmed || device_state == DeviceState::MetadataRequested
					|| device_state == DeviceState::DataRequested) {
				device_state = DeviceState::DataRequested;
				const FlightDataRequest &req = parsed.flight_data_request;
				BeginTransfer(req.record);
				// If the archive record is missing or empty, BeginTransfer leaves
				// transfer_active_ false.  Fall back to MetadataRequested so the
				// app can pick a different record without needing to disarm.
				if (!transfer_active_)
					device_state = DeviceState::MetadataRequested;
			}
			break;
		}

		case MsgType::FlightDataAck: {
			FlightDataAck ack { };
			std::memcpy(&ack, payload, sizeof(ack));
			OnAckReceived(ack, device_state);
			break;
		}

		case MsgType::DeploymentTestRequest: {
			if (flight_state == FlightStates::WaitingLaunch) {
				uint8_t channel = payload[sizeof(PacketHeader)];
				if (channel >= 1 && channel <= 4) {
					deploy_.ResetTestDeployment();
					deploy_.SetActiveDeploymentChannel(channel);
					device_state = DeviceState::Test;
				}
			}
			break;
		}

		default:
			break;
		}
	}

	RgbLed(RgbColor::Off);
}

// ============================================================================
//  Reliable transfer — setup
// ============================================================================

void Communication::BeginTransfer(uint8_t record_id) {
	record_id_ = record_id;
	transfer_active_ = false;
	complete_ = false;
	total_samples_ = 0;
	chunk_buf_count_ = 0;
	chunk_start_ = 0;

	// Query the exact sample count upfront so that total_samples_ and
	// packet_count_ are correct on every wire packet from the first TX.
	uint32_t sample_count = 0;
	if (!archive_.GetFlightSampleCount(record_id, sample_count) || sample_count == 0)
		return;  // record doesn't exist or is empty — silently abort

	total_samples_ = sample_count;
	const size_t spp = FlightProfileCodec::MaxSamplesPerPacket();
	packet_count_ = static_cast<uint16_t>((total_samples_ + spp - 1) / spp);
	if (packet_count_ > kMaxPackets)
		packet_count_ = kMaxPackets;

	// Prime the chunk buffer with the first chunk so SendDataPacket never
	// cold-fetches on the first call. Mirrors startTestReplay() -> fetchNextChunk().
	if (!FetchChunk(0) || chunk_buf_count_ == 0)
		return;  // archive read failed — abort

	// Wrap transfer ID; 0 is reserved as "no transfer"
	if (++transfer_id_ == 0)
		transfer_id_ = 1;

	std::memset(sent_, 0, sizeof(sent_));
	std::memset(acked_, 0, sizeof(acked_));
	std::memset(last_tx_time_, 0, sizeof(last_tx_time_));
	std::memset(parity_acc_, 0, sizeof(parity_acc_));

	window_start_ = 0;
	transfer_active_ = true;
	transfer_ready_ms_ = HAL_GetTick() + kPreTransferGuardMs;
	flight_profile_active_ms_ = HAL_GetTick();
}

// ============================================================================
//  Chunked archive read — mirrors Navigation::fetchNextChunk()
// ============================================================================

bool Communication::FetchChunk(uint32_t first_sample) {
	uint32_t got = 0u;
	const bool ok = archive_.ReadFlightDataRange(record_id_, first_sample, chunk_buf_, kChunkSize, got);

	if (!ok) {
		chunk_buf_count_ = 0;
		return false;
	}

	chunk_buf_count_ = got;
	chunk_start_ = first_sample;
	return got > 0;
}

// ============================================================================
//  Reliable transfer — low-level send helpers
// ============================================================================

bool Communication::TrySendPacket(const FlightDataPacket &pkt, size_t size) {
	const uint32_t now = HAL_GetTick();
	if (radio_busy_)
		return false;
	if (now - last_radio_tx_end_ms_ < kMinRxWindowMs)
		return false;

	radio_busy_ = true;
	radio_->Send(reinterpret_cast<const uint8_t*>(&pkt), size);
	return true;
}

void Communication::SendDataPacket(uint16_t packet_index, uint32_t now_ms) {
	const size_t spp = FlightProfileCodec::MaxSamplesPerPacket();
	const uint32_t global_start = static_cast<uint32_t>(packet_index) * spp;

	// Ensure the chunk covering global_start is loaded.
	// Because kChunkSize is a whole multiple of spp, a packet's samples
	// always fall entirely within one chunk — no packet spans a chunk boundary.
	if (global_start < chunk_start_ || global_start >= chunk_start_ + chunk_buf_count_) {
		// Align the fetch to the chunk boundary that contains global_start.
		// Dividing by kChunkSize and multiplying back gives the aligned start.
		const uint32_t aligned = (global_start / kChunkSize) * kChunkSize;
		if (!FetchChunk(aligned) || chunk_buf_count_ == 0)
			return;  // archive read failed — skip this packet
	}

	const uint32_t chunk_end = chunk_start_ + chunk_buf_count_;
	const uint32_t samples_left = (global_start < chunk_end) ? (chunk_end - global_start) : 0u;
	if (samples_left == 0)
		return;

	const size_t count = (samples_left < spp) ? static_cast<size_t>(samples_left) : spp;

	// Pointer into chunk buffer at the right offset
	const uint32_t local_offset = global_start - chunk_start_;
	const Sample *src = &chunk_buf_[local_offset];

	// Build and send the packet
	FlightDataPacket pkt { };
	pkt.packet_header.system_id = system_id;
	pkt.packet_header.msg_type = MsgType::FlightData;
	pkt.packet_header.msg_count = next_msg_count_++;
	pkt.packet_header.crc = 0;

	pkt.transfer_id = transfer_id_;
	pkt.packet_index = packet_index;
	pkt.packet_count = packet_count_;
	pkt.total_samples = total_samples_;

	const size_t written = FlightProfileCodec::PackSamples(src, count, pkt.payload, sizeof(pkt.payload));

	// Compute the actual serialised payload size from the codec output.
	// PackSamples returns a sample count; the wire layout is:
	//   CompressedHeader + (written-1) * CompressedDelta
	const size_t payload_used = sizeof(FlightProfileCodec::CompressedHeader)
			+ (written > 1 ? (written - 1) * sizeof(FlightProfileCodec::CompressedDelta) : 0);

	const size_t msg_size = offsetof(FlightDataPacket, payload) + payload_used;
	pkt.packet_header.crc = ComputeMessageCrcPartial(reinterpret_cast<const uint8_t*>(&pkt), msg_size);

	if (TrySendPacket(pkt, msg_size)) {
		sent_[packet_index] = true;
		last_tx_time_[packet_index] = now_ms;

		// Accumulate this packet's payload into the parity group buffer.
		// XOR only the bytes actually written; the rest remain zero.
		const uint16_t group = packet_index / kParityGroupSize;
		ParityAccumulator &acc = parity_acc_[group];
		const size_t xor_len = (payload_used < kPayloadSize) ? payload_used : kPayloadSize;
		for (size_t b = 0; b < xor_len; ++b)
			acc.payload[b] ^= pkt.payload[b];
		acc.count++;
	}
}

void Communication::SendParityPacket(uint16_t group_index, uint32_t now_ms) {
	const ParityAccumulator &acc = parity_acc_[group_index];

	FlightDataPacket parity { };
	parity.packet_header.system_id = system_id;
	parity.packet_header.msg_type = MsgType::FlightDataParity;
	parity.packet_header.msg_count = next_msg_count_++;
	parity.packet_header.crc = 0;

	parity.transfer_id = transfer_id_;
	parity.packet_index = group_index;
	parity.packet_count = packet_count_;
	parity.total_samples = total_samples_;

	// Copy the pre-computed XOR payload; kPayloadSize matches pkt.payload exactly.
	std::memcpy(parity.payload, acc.payload, kPayloadSize);

	const size_t msg_size = kMaxPayloadBytes;
	parity.packet_header.crc = ComputeMessageCrcPartial(reinterpret_cast<const uint8_t*>(&parity), msg_size);

	// Parity is best-effort; no ACK tracking.
	TrySendPacket(parity, msg_size);
}

// ============================================================================
//  Reliable transfer — ACK handling
// ============================================================================

bool Communication::AllAcked() const {
	for (uint16_t i = 0; i < packet_count_; ++i)
		if (!acked_[i])
			return false;
	return true;
}

void Communication::OnAckReceived(const FlightDataAck &ack, DeviceState &device_state) {
	if (ack.transfer_id != transfer_id_)
		return;

	for (uint16_t i = 0; i < ack.packet_count; ++i) {
		const uint16_t byte_idx = i / 8;
		const uint8_t bit_idx = i % 8;
		if (ack.bitmap[byte_idx] & (1u << bit_idx))
			acked_[i] = true;
	}

	while (window_start_ < packet_count_ && acked_[window_start_])
		++window_start_;

	flight_profile_active_ms_ = HAL_GetTick();

	if (AllAcked()) {
		complete_ = true;
		// Stay in DataRequested so PreLaunchData is not resumed until the app
		// explicitly exits the flight profile screen (sends DisarmRequest) or
		// the idle timeout fires.
	}
}

// ============================================================================
//  Reliable transfer — main driver (call from main loop / task)
// ============================================================================

void Communication::Process() {
	if (!transfer_active_ || complete_)
		return;
	uint32_t now_ms = HAL_GetTick();
	if (now_ms < transfer_ready_ms_)
		return;  // wait for guard to elapse

	const uint16_t window_end =
			(window_start_ + kWindowSize < packet_count_) ?
					static_cast<uint16_t>(window_start_ + kWindowSize) : packet_count_;

	for (uint16_t i = window_start_; i < window_end; ++i) {
		if (!acked_[i] && !sent_[i]) {
			SendDataPacket(i, now_ms);

			// Emit parity when the last packet in a group is sent, or when
			// the last packet in the whole transfer is sent.
			const uint16_t group = i / kParityGroupSize;
			const uint16_t pos_in_group = i % kParityGroupSize;
			const bool last_in_group = (pos_in_group == kParityGroupSize - 1);
			const bool last_packet = (i == packet_count_ - 1);

			if ((last_in_group || last_packet) && parity_acc_[group].count > 0) {
				SendParityPacket(group, now_ms);
				std::memset(&parity_acc_[group], 0, sizeof(parity_acc_[group]));
			}

			if (radio_busy_)
				break;
		}
	}

	// Expire sent flags so timed-out packets are retransmitted next cycle.
	for (uint16_t i = window_start_; i < window_end; ++i) {
		if (sent_[i] && !acked_[i] && (now_ms - last_tx_time_[i] > kRetxTimeoutMs)) {
			sent_[i] = false;
			// Clear parity accumulator so it is rebuilt from retransmitted data.
			const uint16_t group = i / kParityGroupSize;
			std::memset(&parity_acc_[group], 0, sizeof(parity_acc_[group]));
		}
	}
}

// ============================================================================
//  Frame parser
// ============================================================================

ParseResult Communication::ParseLoraFrame(const uint8_t *data, std::size_t len, uint8_t expected_system_id,
		ParsedMessage &out) {
	if (len < sizeof(PacketHeader))
		return ParseResult::TooShort;

	PacketHeader hdr { };
	std::memcpy(&hdr, data, sizeof(PacketHeader));

	if (hdr.system_id != expected_system_id)
		return ParseResult::SystemIdMismatch;

	if (!ValidateCRC(data, len))
		return ParseResult::CrcMismatch;

	switch (hdr.msg_type) {
	case MsgType::LocatorCfgChgRequest:
		return decode_message<MsgType::LocatorCfgChgRequest>(data, len, out);
	case MsgType::FlightDataAck:
		return decode_message<MsgType::FlightDataAck>(data, len, out);
	case MsgType::FlightDataRequest:
		return decode_message<MsgType::FlightDataRequest>(data, len, out);
	case MsgType::DeploymentTestRequest:
		return decode_message<MsgType::DeploymentTestRequest>(data, len, out);
	case MsgType::ArmRequest:
	case MsgType::DisarmRequest:
	case MsgType::FlightMetadataRequest:
		if (len != sizeof(PacketHeader))
			return ParseResult::LengthMismatch;
		out.type = hdr.msg_type;
		return ParseResult::Ok;
	default:
		return ParseResult::UnknownType;
	}
}

void Communication::CheckFlightProfileTimeout(DeviceState &device_state) {
	if (HAL_GetTick() - flight_profile_active_ms_ > kFlightProfileTimeoutMs)
		device_state = DeviceState::Disarmed;
}

} // namespace Communication
