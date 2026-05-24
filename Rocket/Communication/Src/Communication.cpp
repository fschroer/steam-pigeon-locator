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

namespace Communication {

namespace FA = FlightArchive;

using Sample = FA::FlightSample;
using MsgType = MsgType;

static constexpr size_t SamplesPerPacket = (kMaxPayloadBytes - sizeof(PacketHeader)) / sizeof(Sample);

Communication::Communication(FlightManager &flight, RocketNav::Navigation &nav, Archive &archive,
		PowerManagement &power, Deployment &deploy) :
		flight_(flight), nav_(nav), archive_(archive), power_(power), deploy_(deploy) {
}

void Communication::Init(IRadio &radio) {
	radio_ = &radio;
	radio_->SetChannel(902300000 + archive_.GetLocatorSettings().lora_channel * 200000);
	StartupMessage msg;
	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::Startup;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0; // must be zeroed before computing
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

void Communication::SendPreLaunchData() {
	PreLaunchData msg;
//	NavSolution nav_solution = nav_.getFused();
	GpsSample gps_sample = nav_.getRawGps();
	BaroSample baro_sample = nav_.getRawBaro();
	ImuSample imu_sample = nav_.getRawImu();
	RocketPersistentSettings rocket_settings = archive_.GetLocatorSettings();

	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::PreLaunchData;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0; // must be zeroed before computing

//	msg.latitude = nav_solution.pos.lat_rad * RAD2DEG;
	msg.latitude = gps_sample.lat_rad * RAD2DEG;
//	msg.longitude = nav_solution.pos.lon_rad * RAD2DEG;
	msg.longitude = gps_sample.lon_rad * RAD2DEG;
	msg.satellites = gps_sample.num_sv;
	msg.hacc = gps_sample.h_acc_m;
	msg.imu_status = nav_.imuStatus().health;
	msg.baro_status = nav_.baroStatus().health;
	msg.gps_status = nav_.gpsStatus().health;
	msg.deploy_status = DeploymentChannelContinuity();
//	msg.agl = nav_solution.altitude_agl_m;
	msg.agl = baro_sample.altitude_m_agl;
//	msg.accel = nav_solution.body_accel_mps2;
	msg.accel = imu_sample.accel_selected_mps2;
//	msg.gyro = nav_solution.body_rates_rps;
	msg.gyro = imu_sample.gyro_rps;
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
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(PreLaunchData));
}

void Communication::SendTelemetryData() {
	TelemetryData msg;
//	NavSolution nav_solution = nav_.getFused();
	GpsSample gps_sample = nav_.getRawGps();
	BaroSample baro_sample = nav_.getRawBaro();
	ImuSample imu_sample = nav_.getRawImu();

	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::TelemetryData;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0; // must be zeroed before computing

//	msg.latitude = nav_solution.pos.lat_rad * RAD2DEG;
	msg.latitude = gps_sample.lat_rad * RAD2DEG;
//	msg.longitude = nav_solution.pos.lon_rad * RAD2DEG;
	msg.longitude = gps_sample.lon_rad * RAD2DEG;
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
//	msg.agl = nav_solution.altitude_agl_m;
	msg.agl = baro_sample.altitude_m_agl;
//	msg.accel = nav_solution.body_accel_mps2;
	msg.accel = imu_sample.accel_selected_mps2;
//	msg.gyro = nav_solution.body_rates_rps;
	msg.gyro = imu_sample.gyro_rps;
//	msg.velocity = nav_solution.vertical_speed_mps;
	msg.velocity = gps_sample.vel_d_mps;
	msg.flight_state = flight_.GetFlightState();

	msg.packet_header.crc = ComputeMessageCrc(msg);
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(TelemetryData));
}

void Communication::SendTestCountdownMessage(uint16_t test_deploy_count) {
	DeploymentTestCountdownMessage msg { };
	msg.packet_header.system_id = system_id;
	msg.packet_header.msg_type = MsgType::DeploymentTest;
	msg.packet_header.msg_count = 0;
	msg.packet_header.crc = 0; // must be zeroed before computing

	msg.count = test_deploy_count / SAMPLES_PER_SECOND;
	msg.packet_header.crc = ComputeMessageCrc(msg);
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(DeploymentTestCountdownMessage));
}

void Communication::SendFlightProfileMetadata() {
	FlightMetadata msg;
	bool present = false;
	for (uint8_t i = 0; i < record_count; i++) {
		archive_.ReadEvent(i, FlightArchive::ExampleStatId::FlightTimestampS, msg.record[i].timestamp, present);
		archive_.ReadEvent(i, FlightArchive::ExampleStatId::ApogeeTimestampMs, msg.record[i].apogee, present);
		archive_.ReadEvent(i, FlightArchive::ExampleStatId::LandingTimestampMs, msg.record[i].flight_time, present);
	}
	radio_->Send(reinterpret_cast<uint8_t*>(&msg), sizeof(FlightMetadata));
}

void Communication::SendFlightProfileData() {
	struct FlightProfileMessage {
		PacketHeader packet_header;
		Sample flight_sample[SamplesPerPacket];
	};
}

void Communication::OnRadioTxDone() {
	radio_busy_ = false;
	last_tx_end_ms_ = HAL_GetTick();   // your system tick
}

void Communication::OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo,
		DeviceState &device_state) {
	HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_RESET);

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
			if (flight_state == FlightStates::WaitingLaunch || flight_state == FlightStates::Landed) // Only allow disarm before/after flight
				device_state = DeviceState::Disarmed;
			break;
		case MsgType::FlightMetadataRequest:
			// To do: process flight metadata request
			break;
		case MsgType::FlightDataRequest: {
			//uint8_t flight_record = payload[sizeof(PacketHeader)];
			// To do: read flight record data, transmit
			break;
		}
		case MsgType::FlightDataAck: {
			FlightDataAck ack { };
			std::memcpy(&ack, payload, sizeof(ack));
			OnAckReceived(ack);
			break;
		}
		case MsgType::DeploymentTestRequest: {
			if (flight_state == FlightStates::WaitingLaunch) { // Only allow test before flight
				uint8_t channel = payload[sizeof(PacketHeader)];
				if (channel >= 1 && channel <= 4) {
					deploy_.ResetTestDeployment();
					deploy_.SetActiveDeploymentChannel(channel);
					device_state = DeviceState::Test;
				}
			}
		}
			break;
		default:
			break;
		}
//		GPIO_PinState usb_connected = HAL_GPIO_ReadPin(POWER_SENSE_GPIO_Port, POWER_SENSE_Pin);
	}
	HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
}

void Communication::BeginTransfer(const Sample *samples, uint32_t total_samples) {
	samples_ = samples;
	total_samples_ = total_samples;

	transfer_id_++;
	if (transfer_id_ == 0)
		transfer_id_ = 1;

	std::memset(sent_, 0, sizeof(sent_));
	std::memset(acked_, 0, sizeof(acked_));
	std::memset(last_tx_time_, 0, sizeof(last_tx_time_));

	const size_t max_samples_per_packet = FlightProfileCodec::MaxSamplesPerPacket();

	packet_count_ = (total_samples_ + max_samples_per_packet - 1) / max_samples_per_packet;

	if (packet_count_ > kMaxPackets)
		packet_count_ = kMaxPackets;

	window_start_ = 0;
	complete_ = false;
}

bool Communication::TrySendPacket(const FlightDataPacket &pkt, size_t size) {
	uint32_t now = HAL_GetTick();

	if (radio_busy_)
		return false;

	if (now - last_tx_end_ms_ < kMinRxWindowMs)
		return false;

	radio_busy_ = true;
	radio_->Send(reinterpret_cast<const uint8_t*>(&pkt), size);
	return true;
}

void Communication::SendDataPacket(uint16_t packet_index, uint32_t now_ms) {
	FlightDataPacket pkt { };
	pkt.packet_header.system_id = 1;
	pkt.packet_header.msg_type = MsgType::FlightData;
	pkt.packet_header.msg_count = next_msg_count_++;
	pkt.packet_header.crc = 0;

	pkt.transfer_id = transfer_id_;
	pkt.packet_index = packet_index;
	pkt.packet_count = packet_count_;
	pkt.total_samples = total_samples_;

	const size_t max_samples_per_packet = FlightProfileCodec::MaxSamplesPerPacket();
	const size_t start = packet_index * max_samples_per_packet;
	const size_t remaining = total_samples_ - start;
	const size_t count = (remaining < max_samples_per_packet) ? remaining : max_samples_per_packet;

	const size_t written = FlightProfileCodec::PackSamples(&samples_[start], count, pkt.payload, sizeof(pkt.payload));

	const size_t payload_used = sizeof(FlightProfileCodec::CompressedHeader)
			+ (written > 1 ? (written - 1) * sizeof(FlightProfileCodec::CompressedDelta) : 0);

	const size_t msg_size = sizeof(PacketHeader) + 2u + 2u + 2u + 4u + payload_used;

	pkt.packet_header.crc = ComputeMessageCrcPartial(reinterpret_cast<const uint8_t*>(&pkt), msg_size);

	if (TrySendPacket(pkt, msg_size)) {
		sent_[packet_index] = true;
		last_tx_time_[packet_index] = now_ms;
	}
}

void Communication::SendParityPacket(uint16_t group_index, const FlightDataPacket group[4], uint32_t now_ms) {
	FlightDataPacket parity { };
	parity.packet_header.system_id = 1;
	parity.packet_header.msg_type = MsgType::FlightDataParity;
	parity.packet_header.msg_count = next_msg_count_++;
	parity.packet_header.crc = 0;

	parity.transfer_id = transfer_id_;
	parity.packet_index = group_index;
	parity.packet_count = packet_count_;
	parity.total_samples = total_samples_;

	memset(parity.payload, 0, sizeof(parity.payload));

	for (int i = 0; i < 4; ++i)
		for (size_t b = 0; b < sizeof(parity.payload); ++b)
			parity.payload[b] ^= group[i].payload[b];

	const size_t msg_size = kMaxPayloadBytes;

	parity.packet_header.crc = ComputeMessageCrcPartial(reinterpret_cast<const uint8_t*>(&parity), msg_size);

	TrySendPacket(parity, msg_size);
}

bool Communication::AllAcked() const {
	for (uint16_t i = 0; i < packet_count_; ++i)
		if (!acked_[i])
			return false;
	return true;
}

void Communication::OnAckReceived(const FlightDataAck &ack) {
	if (ack.transfer_id != transfer_id_)
		return;

	for (uint16_t i = 0; i < ack.packet_count; ++i) {
		const uint16_t byte = i / 8;
		const uint8_t bit = i % 8;
		if (ack.bitmap[byte] & (1u << bit))
			acked_[i] = true;
	}

	while (window_start_ < packet_count_ && acked_[window_start_])
		++window_start_;

	if (AllAcked())
		complete_ = true;
}

void Communication::Process(uint32_t now_ms) {
	if (complete_)
		return;

	const uint16_t window_end =
			(window_start_ + kWindowSize < packet_count_) ? window_start_ + kWindowSize : packet_count_;

	FlightDataPacket parity_group[4];
	bool parity_ready = false;
	uint16_t group_index = 0;

	for (uint16_t i = window_start_; i < window_end; ++i) {
		if (!acked_[i] && !sent_[i]) {
			SendDataPacket(i, now_ms);

			uint16_t gi = i / kParityGroupSize;
			uint16_t pos = i % kParityGroupSize;
			parity_group[pos] = { }; // store built packet
			parity_ready = (pos == kParityGroupSize - 1);
			group_index = gi;
		}
	}

	if (parity_ready)
		SendParityPacket(group_index, parity_group, now_ms);

	for (uint16_t i = window_start_; i < window_end; ++i) {
		if (sent_[i] && !acked_[i] && now_ms - last_tx_time_[i] > kRetxTimeoutMs) {
			sent_[i] = false;
		}
	}
}

ParseResult Communication::ParseLoraFrame(const uint8_t* data,
                                          std::size_t len,
                                          uint8_t expected_system_id,
                                          ParsedMessage& out)
{
    using namespace Communication;

    if (len < sizeof(PacketHeader))
        return ParseResult::TooShort;

    // Extract header
    PacketHeader hdr{};
    std::memcpy(&hdr, data, sizeof(PacketHeader));

    // System ID check
    if (hdr.system_id != expected_system_id)
        return ParseResult::SystemIdMismatch;

    // CRC check
    if (!ValidateCRC(data, len))
        return ParseResult::CrcMismatch;

    // Dispatch by message type
    switch (hdr.msg_type)
    {
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

} // namespace Communication
