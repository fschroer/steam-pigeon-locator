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
//
// Kept a whole multiple of kParityGroupSize so each burst contains complete
// parity groups (window 8 = 2 groups).  Larger windows amortise the per-ACK
// overhead (fewer ACK round-trips per transfer) at the cost of a longer burst,
// which is why kRetxTimeoutMs is sized against it.
static constexpr uint16_t kWindowSize = 8;
static constexpr uint16_t kParityGroupSize = 4;

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
	FlightDataAck = 12, // Profile data acknowledgment sent from the app via the receiver.
	DeploymentTestRequest = 13, // Request from the app, via the receiver, for the locator to execute a deployment test.
	DeploymentTest = 14, // Deployment test countdown sent from the locator to the app via the receiver.
	VersionRequest = 17, // Request from the app, via the receiver, for both firmware versions.
	VersionInfo = 18,    // Response: locator version forwarded through receiver, which appends its own version.
	FlightEvents = 19,   // Per-record flight event summary sent alongside a FlightData transfer.
	// Reserved, never sent or parsed here.  The channel survey (ADR-0019 tier 3)
	// is app<->receiver only and the locator plays no part in it — but the MsgType
	// space is shared across all three copies, so the values are claimed here to
	// stop a future locator message silently colliding with them on the wire.
	// Adding these changes no behavior: an already-flashed locator stays compatible.
	ChannelSurveyRequest = 20,
	ChannelSurvey = 21,
	// Suppress the prepped-and-disarmed alert for a bounded time (#37).  A
	// rocket assembled vertically with charges wired is physically identical to
	// one standing on the pad, so no sensor can tell them apart — this is the
	// operator saying "still prepping". Addressed like every other command
	// (ADR-0020): snoozing somebody else's rocket would be a safety hole.
	PadAlertSnoozeRequest = 22,
	// Reserved for the same reason as 20/21 above: the locator search is
	// app<->receiver only, and claiming the values here stops a future locator
	// message colliding with them.  Behavior-free — nothing here sends or parses these.
	LocatorSearchRequest = 23,
	LocatorSearchResult = 24
};

// Flight event summary indices.  One entry per archived event timestamp; the
// order is the wire order of FlightEventsMessage::event_timestamp_ms and MUST
// match the app's FlightEventIndex (RocketState.kt).
enum class FlightEvent : uint8_t {
	Launch = 0,
	Burnout,
	Apogee,
	Noseover,
	DroguePrimaryDeploy,
	DrogueBackupDeploy,
	DrogueVelocityThreshold,
	MainPrimaryDeploy,
	MainBackupDeploy,
	MainVelocityThreshold,
	Landing,
	Count
};
constexpr size_t kFlightEventCount = static_cast<size_t>(FlightEvent::Count);

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
	// Nose axis (ADR-0021 Decision 6, #36).  Broadcast because the app builds its
	// LocatorConfig from THIS message and sends the whole struct back on any
	// config change — a setting the app cannot see is a setting it silently
	// resets to Auto the next time the user edits anything else.
	NoseAxis nose_axis;
	// Explicit arm state (ADR-0021 Decision 3, #35).  Carried here as well as in
	// TelemetryData so the app never has to infer arm state from WHICH message
	// arrived — the inference it used to make, and which ADR-0021 breaks: once
	// arming gates pyro only (#36), a DISARMED locator broadcasts in-flight
	// telemetry, and a type-based reading would report it ARMED.  Sits inside the
	// authenticated region (before locator_id/auth_tag) so the tag covers it.
	uint8_t armed;         // 0 = disarmed, 1 = armed
	// Prepped-and-disarmed verdict (ADR-0021 Decision 5, #37): a rocket standing
	// vertical and still, with deployment-channel continuity, that is not armed.
	//
	// Computed HERE rather than in the app, and broadcast as a verdict, because
	// the app only sees this message at 1 Hz — far too sparse to judge rotational
	// quiescence honestly, and it would have to re-derive verticality from an
	// accel snapshot without the locator's mounting frame.  One implementation at
	// 20 Hz also guarantees the buzzer and the app alert agree, which is the whole
	// point of the two channels being independent but not divergent.
	// 0 = quiet, 1 = alerting, 2 + n = snoozed with n minutes left (n rounds up,
	// so 2 means under a minute remains and 17 is the 15-minute ceiling).
	//
	// The remaining time rides in the same byte rather than taking a new one:
	// the app needs it to show what repeated taps have accumulated, and a value
	// an old app cannot decode still reads as non-zero — i.e. as "alerting",
	// which is the safe direction for anything it does not understand.
	uint8_t pad_alert;
	uint32_t locator_id;   // cleartext STM MPU UID (== DeviceUID::getUID()); app identifies the locator by this
	uint32_t auth_tag;     // password-seeded checksum (see Communication::ComputePasswordAuthTag); 0 while computing
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
	// Explicit arm state (ADR-0021 Decision 3, #35).  The app previously read
	// "TelemetryData ⇒ armed" off the message type; ADR-0021 invalidates that,
	// because a disarmed locator will broadcast telemetry in flight (#36) and an
	// unarmed ballistic flight would then display as ARMED — suppressing exactly
	// the warning the operator needs.  Inside the authenticated region so the tag
	// covers it: a safety indicator must not be forgeable by a bystander.
	uint8_t armed;        // 0 = disarmed, 1 = armed
	// Identity + authenticator, mirroring PreLaunchData's trailing pair and
	// computed the same way (ADR-0006).  Carried here too so the app can
	// recognize an ARMED locator: an armed locator sends nothing but
	// TelemetryData, so without these the app has no way to tell whose telemetry
	// it is holding — it either shows an unauthenticated stream or, as it did,
	// shows nothing at all until the locator is disarmed.
	//
	// auth_tag MUST remain the last field: both firmwares and the app compute the
	// tag over the base struct with packet_header.crc and the trailing 4 bytes
	// zeroed, and locate it by offset from the end.
	uint32_t locator_id;   // cleartext STM MPU UID (== DeviceUID::getUID())
	uint32_t auth_tag;     // password-seeded checksum; 0 while computing
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

// Per-record flight event summary.  The FlightMetadata list message carries only
// what is needed to *identify* a record (timestamp/apogee/flight time) for all
// record_count slots; a full event set for every slot would not fit one LoRa
// frame.  This message instead describes the ONE record the app has asked for,
// and the locator sends it alongside the FlightData transfer for that record.
//
// Only event *times* are sent.  Event altitudes are not: the app derives them by
// looking up the flight sample nearest each timestamp in the profile data it
// receives, which keeps the two in exact agreement on the chart.
struct FlightEventsMessage {
	PacketHeader packet_header;
	uint8_t  record;              // archive slot this summary describes
	uint8_t  reserved;            // pad to keep the u16/u32 fields naturally sized
	uint16_t present_mask;        // bit (1 << FlightEvent) set ⇒ that timestamp is valid
	uint32_t flight_timestamp_s;  // GPS wall clock at flight start (Statistic::FlightTimestampS)
	uint32_t event_timestamp_ms[kFlightEventCount];  // ms since the launch epoch, indexed by FlightEvent
	float    max_altitude_m;      // raw-baro apogee peak (Statistic::MaxAltitudeM)
	uint8_t  deployment_ch_stats[4];   // per channel: mode (bits 0-2) | fired (3) | pre-fire cont. (4) | post-fire cont. (5)
};
// NOTE: Statistic::MaxVelocityMps, MaxAccelerationMps2 and PhysicalDeploymentStats
// exist in the archive enum but are never written by FlightManager, so they are
// deliberately absent here — sending them would ship guaranteed zeros.  Add them
// once the locator actually records them (and bump the size assert below).

// On-wire packet for flight profile transfer
struct FlightDataPacket {
	PacketHeader packet_header;

	uint16_t transfer_id;   // identifies this flight profile transfer
	uint16_t packet_index;  // 0..packet_count-1 (data) or parity index
	uint16_t packet_count;  // total data packets (excluding parity)
	uint32_t total_samples; // total samples in transfer
	uint8_t payload[kPayloadSize]; // Compressed payload bytes
};

// ── Addressed app→locator commands (ADR-0020, #34) ──────────────────────────
//
// The receiver relays these over LoRa, which is a broadcast medium, so without a
// target EVERY locator on the channel in an accepting state obeys them.  On the
// bench that rewrote a bystander locator's whole RocketPersistentSettings —
// deployment modes, deploy delays, main altitudes — and the same applies to
// ArmRequest, which would arm every disarmed locator on a shared launch channel.
//
// target_locator_id is the MCU UID the app already receives in every broadcast
// (ADR-0006), so no new identity concept is introduced.  It sits immediately
// after the header on every command, so the locator reads it at one fixed offset.
// **0 matches nothing** — on a path that includes Arm the failure direction must
// be "do nothing", so an unaddressed frame from an old app is discarded.
struct TargetedRequest {          // ArmRequest, DisarmRequest,
	PacketHeader packet_header;   // FlightMetadataRequest, VersionRequest
	uint32_t target_locator_id;
};

struct LocatorSettings {
	PacketHeader header;
	uint32_t target_locator_id;
	RocketPersistentSettings settings;
};

struct FlightDataAck {
	PacketHeader header;
	uint32_t target_locator_id;

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
	uint32_t target_locator_id;
	uint8_t record;
};

struct DeploymentTestRequest {
	PacketHeader packet_header;
	uint32_t target_locator_id;
	uint8_t channel;       // 1-4 starts a test; 0 CANCELS a running one
};

// Bounded suppression of the #37 alert.  Duration is carried rather than fixed
// so the policy lives in the app, and it is BOUNDED on purpose — a snooze that
// never expires is just an off switch, and would reintroduce the forgotten arm
// this whole ADR exists to catch.  RAM-only on the locator: a power cycle
// clears it, which fails in the safe direction.
struct PadAlertSnoozeRequest {
	PacketHeader packet_header;
	uint32_t target_locator_id;
	uint8_t minutes;       // 0 cancels an active snooze
};

// NOTE: there is deliberately no "is this command addressed?" predicate here.
// The locator requires an address from EVERY frame it acts on (ADR-0020), because
// every frame reaching it is an app command.  An allowlist would have to be
// updated by hand for each new command, and forgetting would silently restore the
// pre-ADR-0020 behavior where one command reached every rocket on the channel.
// The receiver still sizes each command individually — see its message_length_
// table — but that is a per-type length, not a policy list.

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

// ─── Wire-layout cross-check (issue #4) ──────────────────────────────────────
// Pin the on-wire size of every message struct so any field change fails the
// firmware build until the literal is updated — a reminder to update the matching
// Kotlin sizes in the app (Protocol.* / FlightDataRepository consts, locked by
// app/src/test/java/com/steampigeon/flightmanager/WireLayoutTest.kt).  Keep these
// literals equal on both sides.  app payload = sizeof(struct) − header(6) [+ any
// receiver-appended bytes].  Catches size drift (field add/remove/resize); a
// same-size field reorder would need offsetof asserts (omitted for portability).
static_assert(sizeof(PacketHeader)                   ==   6, "PacketHeader size changed — sync app Protocol.HEADER_SIZE");
static_assert(kPayloadSize                           == 239, "FlightData payload size changed");
static_assert(sizeof(StartupMessage)                 ==  74, "StartupMessage size changed");
static_assert(sizeof(VersionInfoMessage)             ==  70, "VersionInfoMessage size changed");
static_assert(sizeof(PreLaunchData)                  == 118, "PreLaunchData size changed (app payload 112 = 101 + nose_axis 1 + armed 1 + pad_alert 1 + locator_id 4 + auth_tag 4)");
static_assert(sizeof(TelemetryData)                  ==  77, "TelemetryData size changed (app payload 71 = 62 + armed 1 + locator_id 4 + auth_tag 4; + rssi 2 = 73)");
static_assert(sizeof(FlightMetadataRecord)           ==  10, "FlightMetadataRecord size changed");
// 96, was 106: FlightMetadata carries one FlightMetadataRecord per archive slot,
// and record_count went 10 -> 9 when FlightSample grew for ARCHIVE_VERSION 6
// (#38).  This is an APP-VISIBLE wire change — the flight list will misparse
// until Protocol.* and WireLayoutTest.kt are updated to match.
static_assert(sizeof(FlightMetadata)                 ==  96, "FlightMetadata size changed (app payload 90 = 9 records x 10)");
static_assert(kFlightEventCount                      ==  11, "FlightEvent count changed — sync app FlightEventIndex");
static_assert(sizeof(FlightEventsMessage)            ==  66, "FlightEventsMessage size changed (app payload 60)");
static_assert(sizeof(FlightDataPacket)               == 255, "FlightDataPacket size changed (max LoRa frame 255)");
static_assert(sizeof(RocketPersistentSettings)       ==  35, "RocketPersistentSettings size changed");
static_assert(sizeof(TargetedRequest)                ==  10, "TargetedRequest size changed (ADR-0020) — sync receiver + app");
static_assert(sizeof(LocatorSettings)                ==  45, "LocatorSettings size changed — sync receiver + app");
static_assert(sizeof(FlightDataAck)                  ==  46, "FlightDataAck size changed (app FLIGHT_DATA_ACK_SIZE)");
static_assert(sizeof(DeploymentTestCountdownMessage) ==   7, "DeploymentTestCountdownMessage size changed");
static_assert(sizeof(FlightDataRequest)              ==  11, "FlightDataRequest size changed");
static_assert(sizeof(DeploymentTestRequest)          ==  11, "DeploymentTestRequest size changed");
static_assert(sizeof(PadAlertSnoozeRequest)          ==  11, "PadAlertSnoozeRequest size changed — sync receiver + app");

} // namespace Communication
