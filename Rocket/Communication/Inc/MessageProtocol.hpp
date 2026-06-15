#pragma once
#include <cstddef>
#include <cstdint>

#include "RocketSettings.hpp"
#include "Archive.hpp"

namespace Communication {

// Max total bytes for one LoRa packet.  The radio's payload-length register
// is 8 bits, so the hard limit is 255.  Using 256 wraps to 0 and causes the
// radio to transmit an empty packet.
constexpr size_t kMaxPayloadBytes = 255;
constexpr uint8_t system_id = 0x44;
constexpr uint16_t kCrc16Poly = 0xA001;   // CRC‑16/IBM reflected polynomial
constexpr uint16_t kCrc16Key = 0xFFFF;   // standard initial value
// Burst window size.  The locator sends kWindowSize data packets (plus one
// parity per group of kParityGroupSize) back-to-back, then goes quiet and
// listens.  The receiver defers the cumulative ACK until the burst has been
// silent for kAckDeferMs, so the ACK is only sent once the locator's radio
// is idle and listening.  kRetxTimeoutMs must exceed the full burst time +
// deferral + ACK airtime (see Communication.hpp for the budget).
static constexpr uint16_t kWindowSize = 4;
static constexpr uint16_t kParityGroupSize = 4;

enum class MsgState : uint8_t {
	None = 0, Arm = 1, Disarm = 2, Test = 3, Config = 4, FlightMetadata = 5, FlightData = 6
};

// Message type for the packet header
enum class MsgType : uint8_t {
	Startup = 0, // Initial message at startup
	LocatorCfgChgRequest = 1, // Request to update locator configuration sent from the app via the receiver.
	ReceiverCfgChgRequest = 2, // Request to update receiver configuration sent from the app to the receiver.
	ArmRequest = 3, // Request to arm the locator sent from the app via the receiver.
	DisarmRequest = 4, // Request to disarm the locator sent from the app via the receiver.
	PreLaunchData = 5, // Unsolicited locator status sent from the locator while in an unarmed state.
	TelemetryData = 6, // Unsolicited locator status sent from the locator while in an armed state.
	FlightMetadataRequest = 7, // Request from the app, via the receiver, for high-level information necessary to identify each flight profile record archived by the locator.
	FlightMetadata = 8, // Flight profile metadata response from the locator to the app via the receiver.
	FlightDataRequest = 9, // Request from the app, via the receiver, for the data in one flight profile.
	FlightData = 10, // Flight profile data response from the locator to the app via the receiver consisting of multiple packets, which the app acknowledges via the receiver.
	FlightDataParity = 11, // Parity packet to allow the app to reconstruct profile data if one packet is lost.
	FlightDataAck = 12, // Profile data acknowledgement sent from the app via the receiver.
	DeploymentTestRequest = 13, // Request from the app, via the receiver, for the locator to execute a deployment test.
	DeploymentTest = 14, // Deployment test countdown sent from the locator to the app via the receiver.
	VersionRequest = 17, // Request from the app, via the receiver, for both firmware versions.
	VersionInfo = 18     // Response: locator version forwarded through receiver, which appends its own version.
};

#pragma pack(push, 1)
// Common packet header (on-wire)
struct PacketHeader {
	uint8_t system_id; // 1 byte
	MsgType msg_type;  // 1 byte
	uint16_t msg_count; // 2 bytes
	uint16_t crc;       // 2 bytes (CRC-16 with secret seed)
};

// Compute payload size AFTER PacketHeader is complete
constexpr size_t kPayloadSize = kMaxPayloadBytes - sizeof(PacketHeader)   // header
		- 2u                     // transfer_id
		- 2u                     // packet_index
		- 2u                     // packet_count
		- 4u;                    // total_samples

struct StartupMessage {
	PacketHeader packet_header;
	uint32_t serial_number;
	uint8_t version[64];
};

struct VersionInfoMessage {
	PacketHeader packet_header;
	uint8_t locator_version[64];
};

struct PreLaunchData {
	PacketHeader packet_header;
	double latitude;
	double longitude;
	double raw_latitude;
	double raw_longitude;
	uint8_t satellites;
	float hacc;
	SensorHealth imu_status;
	SensorHealth baro_status;
	SensorHealth gps_status;
	uint8_t deploy_status;
	float agl;
	Vec3f accel;
	Vec3f gyro;
	DeployMode deploy_ch1_mode;
	DeployMode deploy_ch2_mode;
	DeployMode deploy_ch3_mode;
	DeployMode deploy_ch4_mode;
	uint8_t drogue_primary_deploy_delay;
	uint8_t drogue_backup_deploy_delay;
	uint16_t main_primary_deploy_altitude;
	uint16_t main_backup_deploy_altitude;
	char device_name[device_name_length];
	uint16_t battery_voltage_mvolt;
};

struct TelemetryData {
	PacketHeader packet_header;
	double latitude;
	double longitude;
	uint8_t satellites;
	float hacc;
	SensorHealth imu_status;
	SensorHealth baro_status;
	SensorHealth gps_status;
	uint8_t deployment_ch1_stats;
	uint8_t deployment_ch2_stats;
	uint8_t deployment_ch3_stats;
	uint8_t deployment_ch4_stats;
	uint8_t physical_deployment_stats;
	float agl;
	Vec3f vel_ned_mps;    // fused NED velocity (north, east, down) m/s
	Quaternionf q_bn;     // body-to-NED attitude quaternion (w, x, y, z)
	FlightStates flight_state;
};

struct FlightMetadataRecord {
	uint32_t timestamp;
	float apogee;
	uint16_t flight_time;
};

struct FlightMetadata {
	PacketHeader packet_header;
	FlightMetadataRecord record[record_count];
};

// On-wire packet for flight profile transfer
struct FlightDataPacket {
	PacketHeader packet_header;

	uint16_t transfer_id;   // identifies this flight profile transfer
	uint16_t packet_index;  // 0..packet_count-1 (data) or parity index
	uint16_t packet_count;  // total data packets (excluding parity)
	uint32_t total_samples; // total samples in transfer
	uint8_t payload[kPayloadSize]; // Compressed payload bytes
};

struct LocatorSettings {
	PacketHeader header;
	RocketPersistentSettings settings;
};

struct FlightDataAck {
	PacketHeader header;

	uint16_t transfer_id;
	uint16_t packet_count;

	static constexpr uint16_t kMaxPayloadBytes = 256;
	uint8_t bitmap[kMaxPayloadBytes / 8];
};

struct DeploymentTestCountdownMessage {
	PacketHeader packet_header;
	uint8_t count;
};

struct FlightDataRequest {
	PacketHeader packet_header;
	uint8_t record;
};

struct DeploymentTestRequest {
	PacketHeader packet_header;
	uint8_t channel;
};

struct ParsedMessage {
    MsgType type;

    union {
        LocatorSettings locator_settings;
        FlightDataAck flight_data_ack;
        FlightDataRequest flight_data_request;
        DeploymentTestRequest deployment_test_request;
//        PacketHeader packet_header;
    };
};

#pragma pack(pop)

} // namespace Communication
