#pragma once

//extern "C" {
//#include "radio.h"
//#include "usart.h"
//#include "subghz_phy_app.h"
//#include "stm32wlxx_hal_rtc.h"
//#include "tim.h"
//}

#include "DeviceUID.hpp"
#include "FlightManager.hpp"
#include "Navigation.hpp"
#include "Archive.hpp"
#include "PowerManagement.hpp"
#include "ArchiveTypes.hpp"
#include "FlightProfileCodec.hpp"
#include "MessageProtocol.hpp"
#include "Deployment.hpp"

namespace Communication {

#define LORA_MSG_TYPE_SIZE 3
#define FLIGHT_DATA_SEQUENCE_SIZE 1
#define FLIGHT_STATS_MSG_SIZE 81
#define FLIGHT_DATA_MESSAGE_SAMPLES 30

enum class ParseResult {
	Ok, TooShort, SystemIdMismatch, CrcMismatch, LengthMismatch, UnknownType
};

template<typename T>
ParseResult decode_into(const uint8_t *data, std::size_t len, T &out) {
	static_assert(std::is_trivially_copyable_v<T>,
			"Message type must be trivially copyable");

	if (len != sizeof(T))
		return ParseResult::LengthMismatch;

	std::memcpy(&out, data, sizeof(T));
	return ParseResult::Ok;
}

template<MsgType M> struct MsgTraits;

template<> struct MsgTraits<MsgType::LocatorCfgChgRequest> {
	using type = LocatorSettings;
	static constexpr auto field = &ParsedMessage::locator_settings;
};

template<> struct MsgTraits<MsgType::FlightDataAck> {
	using type = FlightDataAck;
	static constexpr auto field = &ParsedMessage::flight_data_ack;
};

template<> struct MsgTraits<MsgType::FlightDataRequest> {
	using type = FlightDataRequest;
	static constexpr auto field = &ParsedMessage::flight_data_request;
};

template<> struct MsgTraits<MsgType::DeploymentTestRequest> {
	using type = DeploymentTestRequest;
	static constexpr auto field = &ParsedMessage::deployment_test_request;
};

template<MsgType M>
ParseResult decode_message(const uint8_t *data, std::size_t len, ParsedMessage &out) {
	auto field = MsgTraits<M>::field;

	auto result = decode_into(data, len, out.*field);
	if (result == ParseResult::Ok)
		out.type = M;

	return result;
}

// Simple radio interface so we don't hide globals
class IRadio {
public:
	virtual ~IRadio() = default;
	virtual void Send(const uint8_t *data, size_t len) = 0;
	virtual void Rx(uint32_t timeout_ms) = 0;
	virtual void SetChannel(uint32_t freq) = 0;
};

class Communication {
public:
	Communication(DeviceUID &deviceUID, FlightManager &flight, RocketNav::Navigation &nav, Archive &archive,
			PowerManagement &power, Deployment &deploy);
	void Init(IRadio &radio);
	void OnRadioTxDone();   // called from ISR/callback
	void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo, DeviceState &device_state);
	void SetChannel(uint8_t channel);
	void SendGenericPacket(const uint8_t *data, size_t len);
	void SendPreLaunchData();
	void SendTelemetryData();
	void SendTestCountdownMessage(uint16_t test_deploy_count);
	void SendFlightProfileMetadata(DeviceState &device_state);
	void SendFlightProfileData();

	// Initiate a reliable chunked transfer of one flight record.
	// Called from OnRadioRxDone on receipt of FlightDataRequest.
	void BeginTransfer(uint8_t record_id);

	// Call periodically from the main loop / task.
	void Process();

	void OnAckReceived(const FlightDataAck &ack, DeviceState &device_state);
	bool IsComplete() const { return complete_; }

private:
	DeviceUID            &deviceUID_;
	FlightManager        &flight_;
	RocketNav::Navigation &nav_;
	Archive              &archive_;
	PowerManagement      &power_;
	IRadio               *radio_ = nullptr;
	Deployment           &deploy_;

	bool send_metadata_ = false;
	uint32_t send_metadata_request_time_ = 0;
	static constexpr uint32_t kPreTransferGuardMs = 50u;
	uint32_t transfer_ready_ms_ = 0;

	// -----------------------------------------------------------------------
	// Transfer protocol constants
	// -----------------------------------------------------------------------
	// Note: kWindowSize and kParityGroupSize are defined at namespace scope
	// in MessageProtocol.hpp and are intentionally not redefined here.

	// Milliseconds before an unacknowledged sent packet is eligible for
	// retransmission.
	static constexpr uint32_t kRetxTimeoutMs = 200;

	// Maximum number of packets in a single transfer.
	// Sized to match the 256-bit ACK bitmap in FlightDataAck.
	static constexpr uint16_t kMaxPackets = 256;

	// Minimum gap between the end of a TX and the next TX, giving the
	// receiver time to process and reply.
	static constexpr uint32_t kMinRxWindowMs = 20;

	// -----------------------------------------------------------------------
	// Chunk buffer — small static window into the archive
	//
	// Rather than loading all flight samples into RAM at once, we keep a
	// small sliding buffer and re-fetch from the archive whenever
	// SendDataPacket needs samples that are outside the current window.
	// This mirrors the NAV_TEST replay pattern in Navigation.cpp exactly.
	//
	// kChunkSize must be a whole multiple of MaxSamplesPerPacket() so that
	// every packet's samples fall entirely within one chunk fetch.
	// With MaxSamplesPerPacket() == 9, we use 9*8 = 72 (8 packets/chunk,
	// ~4 KB of RAM for the buffer).
	// -----------------------------------------------------------------------
	static constexpr uint32_t kChunkSize = 72u;

	// Compile-time guard: if MaxSamplesPerPacket() ever changes and kChunkSize
	// is no longer a whole multiple, this will catch it.
	static_assert(kChunkSize % FlightProfileCodec::MaxSamplesPerPacket() == 0,
			"kChunkSize must be a whole multiple of MaxSamplesPerPacket()");

	// The chunk buffer itself and its bookkeeping — identical in spirit to
	// m_test_buf_, m_test_buf_count_, m_test_buf_index_, m_test_chunk_start_
	// in Navigation.hpp.
	FlightArchive::FlightSample chunk_buf_[kChunkSize]{};
	uint32_t chunk_buf_count_  = 0;  // valid samples currently in chunk_buf_
	uint32_t chunk_start_      = 0;  // global sample index of chunk_buf_[0]

	// -----------------------------------------------------------------------
	// Transfer state
	// -----------------------------------------------------------------------

	uint8_t  record_id_       = 0;
	bool     transfer_active_ = false;
	uint32_t total_samples_   = 0;  // exact count from GetFlightSampleCount(), set in BeginTransfer()

	uint16_t transfer_id_   = 0;
	uint16_t packet_count_  = 0;

	bool     sent_[kMaxPackets]         = {};
	bool     acked_[kMaxPackets]        = {};
	uint32_t last_tx_time_[kMaxPackets] = {};

	// Lowest packet index not yet acknowledged; left edge of the sliding window.
	uint16_t window_start_ = 0;
	bool     complete_     = false;

	// Per-group parity accumulator.  Each entry accumulates the XOR of all
	// FlightDataPacket payloads in a parity group so that the parity packet
	// can be emitted as soon as the last member of the group is sent.
	// kPayloadSize is defined in MessageProtocol.hpp as the usable payload
	// region of a FlightDataPacket (kMaxPayloadBytes minus header and fixed
	// fields), so this exactly matches the on-wire payload array.
	struct ParityAccumulator {
		uint8_t payload[kPayloadSize] = {};
		uint8_t count = 0;  // number of data packets XOR'd in so far
	};
	ParityAccumulator parity_acc_[kMaxPackets / kParityGroupSize];

	// -----------------------------------------------------------------------
	// Radio / sequencing state
	// -----------------------------------------------------------------------

	uint16_t next_msg_count_ = 0;
	bool     radio_busy_     = false;
	uint32_t last_radio_tx_end_ms_ = 0;

	// -----------------------------------------------------------------------
	// Internal helpers
	// -----------------------------------------------------------------------

	// Fetch the chunk that contains global sample index first_sample into
	// chunk_buf_.  Returns false if the archive read fails or yields 0 samples.
	// Mirrors Navigation::fetchNextChunk().
	bool FetchChunk(uint32_t first_sample);

	void SendDataPacket(uint16_t packet_index, uint32_t now_ms);
	void SendParityPacket(uint16_t group_index, uint32_t now_ms);

	bool TrySendPacket(const FlightDataPacket &pkt, size_t size);
	bool AllAcked() const;
	ParseResult ParseLoraFrame(const uint8_t *data, std::size_t len, uint8_t expected_system_id, ParsedMessage &out);

	// -----------------------------------------------------------------------
	// CRC helpers (inlined for speed on the embedded target)
	// -----------------------------------------------------------------------

	inline uint16_t Crc16Update(uint16_t crc, uint8_t data) {
		crc ^= data;
		for (int i = 0; i < 8; i++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
		return crc;
	}

	// Feed a byte range into an already-running CRC (no re-seeding).
	inline uint16_t Crc16Continue(uint16_t crc, const uint8_t *data, size_t len) {
		for (size_t i = 0; i < len; ++i)
			crc = Crc16Update(crc, data[i]);
		return crc;
	}

	// Start a fresh CRC from the keyed seed.
	inline uint16_t Crc16Start(const uint8_t *data, size_t len) {
		return Crc16Continue(kCrc16Key, data, len);
	}

	// Full-message CRC for fixed-size POD messages.
	// Covers: header bytes [0..3] (before the crc field), then all bytes
	// after the PacketHeader.  The crc field itself (bytes 4-5) is skipped.
	template<typename TMsg>
	inline uint16_t ComputeMessageCrc(const TMsg &msg) {
		static_assert(std::is_trivially_copyable<TMsg>::value, "TMsg must be POD");

		const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&msg);
		constexpr size_t crc_field_offset = offsetof(PacketHeader, crc);   // 4

		uint16_t crc = Crc16Start(bytes, crc_field_offset);
		const size_t after_crc = crc_field_offset + sizeof(uint16_t);
		crc = Crc16Continue(crc, bytes + after_crc, sizeof(TMsg) - after_crc);
		return crc;
	}

	// Partial-message CRC for variable-length packets (FlightDataPacket).
	// Same skip rule: bytes [0..3], skip [4..5], then bytes [6..msg_size).
	inline uint16_t ComputeMessageCrcPartial(const uint8_t *bytes, size_t msg_size) {
		constexpr size_t crc_field_offset = offsetof(PacketHeader, crc);   // 4

		uint16_t crc = Crc16Start(bytes, crc_field_offset);
		const size_t after_crc = crc_field_offset + sizeof(uint16_t);
		if (msg_size > after_crc)
			crc = Crc16Continue(crc, bytes + after_crc, msg_size - after_crc);
		return crc;
	}

	inline bool ValidateCRC(const uint8_t *data, std::size_t len) {
		if (len < sizeof(PacketHeader))
			return false;

		const PacketHeader *hdr = reinterpret_cast<const PacketHeader*>(data);
		constexpr size_t crc_field_offset = offsetof(PacketHeader, crc);

		uint16_t crc = Crc16Start(data, crc_field_offset);
		const size_t after_crc = crc_field_offset + sizeof(uint16_t);
		crc = Crc16Continue(crc, data + after_crc, len - after_crc);
		return crc == hdr->crc;
	}
};

} // namespace Communication
