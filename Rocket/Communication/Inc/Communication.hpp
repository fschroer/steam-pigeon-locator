#pragma once

//extern "C" {
//#include "radio.h"
//#include "usart.h"
//#include "subghz_phy_app.h"
//#include "stm32wlxx_hal_rtc.h"
//#include "tim.h"
//}

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

// Simple radio interface so we don't hide globals
class IRadio
{
public:
    virtual ~IRadio() = default;
    virtual void Send(const uint8_t* data, size_t len) = 0;
    virtual void Rx(uint32_t timeout_ms) = 0;
    virtual void SetChannel(uint32_t freq) = 0;
};

class Communication{
public:
  Communication(FlightManager& flight,
  		RocketNav::Navigation& nav,
      Archive& archive,
  		PowerManagement& power,
			Deployment& deploy);
  void Init(IRadio& radio);
//  void Send(uint8_t *buffer, uint8_t size);
  void OnRadioTxDone();   // called from ISR/callback
  void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo, DeviceState& device_state);   // ACK reception handler
  void SetChannel(uint8_t channel);
  void SendGenericPacket(const uint8_t* data, size_t len);
  void SendPreLaunchData();
  void SendTelemetryData();
  void SendTestCountdownMessage(uint16_t test_deploy_count);
  void SendFlightProfileMetadata();
  void SendFlightProfileData();
  void BeginTransfer(const FlightArchive::ExampleFlightSample* samples,
      uint32_t total_samples);

  // Call periodically from main loop or task
  void Process(uint32_t now_ms);

  void OnAckReceived(const FlightProfileAck& ack);
  bool IsComplete() const { return complete_; }

private:
  FlightManager& flight_;
  RocketNav::Navigation& nav_;
  Archive& archive_;
  PowerManagement& power_;
  IRadio* radio_ = nullptr;
  Deployment& deploy_;

  static constexpr uint32_t kRetxTimeoutMs   = 200;
  static constexpr uint16_t kMaxPackets      = 256;
  const FlightArchive::ExampleFlightSample* samples_ = nullptr;
  size_t sample_count_;
  uint32_t total_samples_ = 0;

  uint16_t transfer_id_  = 0;
  uint16_t packet_count_ = 0;
  uint16_t next_packet_index_;
  uint16_t total_packets_;


  bool     sent_[kMaxPackets]      = {};
  bool     acked_[kMaxPackets]     = {};
  uint32_t last_tx_time_[kMaxPackets] = {};

  uint16_t window_start_ = 0;
  bool     complete_     = false;

  uint16_t next_msg_count_ = 0;

  bool radio_busy_ = false;
  uint32_t last_tx_end_ms_ = 0;
  static constexpr uint32_t kMinRxWindowMs = 20;

  void SendDataPacket(uint16_t packet_index, uint32_t now_ms);
  void SendParityPacket(uint16_t group_index,
                        const FlightProfilePacket group[4],
                        uint32_t now_ms);

  bool TrySendPacket(const FlightProfilePacket& pkt, size_t size);
  bool AllAcked() const;

  const char* lora_startup_message_ = "Rocket Locator v1.3.1\r\n\0";
  const char* usb_connected_ = "Disconnect USB cable before arming locator\r\n\0";
  const char* bad_gps_data_ = "Bad GPS Data\r\n\0";

//  uint8_t rocket_service_count_ = 0;
//  int peripheral_interrupt_count_ = 0;
//  int battery_level_ = 0;
//  int flight_stats_delay_count_ = 0;
//
//  bool archive_opened_ = false;
//  bool initial_flight_data_written = false;
//  bool datestamp_saved_ = false;
//  bool ready_to_send_ = true;
//
//  uint8_t flight_profile_archive_position_ = 0;
//  uint8_t flight_profile_packet_index_ = 0;
//  uint8_t flight_profile_wait_count_ = 0;
//
//  uint32_t start_time_ = 0;

  void TransmitLEDsOn();
  void TransmitLEDsOff();
  uint16_t GetBatteryLevel();

  inline uint16_t Crc16Update(uint16_t crc, uint8_t data)
  {
      crc ^= data;
      for (int i = 0; i < 8; i++) {
          if (crc & 1)
              crc = (crc >> 1) ^ 0xA001;
          else
              crc >>= 1;
      }
      return crc;
  }

  inline uint16_t Crc16Keyed(const uint8_t* data, size_t len)
  {
      uint16_t crc = kCrc16Key;   // your secret seed
      for (size_t i = 0; i < len; ++i)
          crc = Crc16Update(crc, data[i]);
      return crc;
  }

  template<typename TMsg>
  inline uint16_t ComputeMessageCrc(const TMsg& msg)
  {
      static_assert(std::is_trivially_copyable<TMsg>::value,
                    "TMsg must be POD");

      const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&msg);

      // 1) First 4 bytes of PacketHeader
      uint16_t crc = kCrc16Key;
      crc = Crc16Update(crc, bytes[0]);
      crc = Crc16Update(crc, bytes[1]);
      crc = Crc16Update(crc, bytes[2]);
      crc = Crc16Update(crc, bytes[3]);

      // 2) Skip CRC field (bytes 4–5)
      // 3) Hash everything after header
      const size_t payload_offset = sizeof(PacketHeader);
      const size_t payload_len    = sizeof(TMsg) - payload_offset;

      for (size_t i = payload_offset; i < sizeof(TMsg); ++i)
          crc = Crc16Update(crc, bytes[i]);

      return crc;
  }

  inline uint16_t ComputeMessageCrcPartial(const uint8_t* bytes, size_t msg_size)
  {
      using FPHeader = PacketHeader;

      // Where the CRC field lives inside the header
      constexpr size_t header_crc_offset = offsetof(FPHeader, crc);
      constexpr size_t header_crc_size   = sizeof(FPHeader) - sizeof(uint16_t);

      // Start with the keyed seed
      uint16_t crc = kCrc16Key;

      // 1) Header bytes BEFORE the CRC field
      crc = Crc16Keyed(bytes, header_crc_offset);

      // 2) Header bytes AFTER the CRC field
      crc = Crc16Keyed(bytes + header_crc_offset + sizeof(uint16_t),
                       header_crc_size - header_crc_offset);

      // 3) Everything after the header
      if (msg_size > sizeof(FPHeader)) {
          const size_t tail_len = msg_size - sizeof(FPHeader);
          crc = Crc16Keyed(bytes + sizeof(FPHeader), tail_len);
      }

      return crc;
  }
};
} // namespace Communication
