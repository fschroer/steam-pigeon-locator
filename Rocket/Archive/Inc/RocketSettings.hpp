#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <cstring>

// DeployMode and NoseAxis.  No cycle: Types.hpp pulls only Constants.hpp.
// (DeployMode was previously relied on being declared by whoever included this
// header first — see the commented-out copy below.)
#include "Types.hpp"

constexpr std::size_t device_name_length = 20;

//enum class DeployMode : uint8_t
//{
//    DroguePrimary = 0,
//    DrogueBackup  = 1,
//    MainPrimary   = 2,
//    MainBackup    = 3
//};
//
#pragma pack(push, 1)

struct RocketPersistentSettings
{
    DeployMode deployment_ch1_mode = DeployMode::DroguePrimary;
    DeployMode deployment_ch2_mode = DeployMode::DrogueBackup;
    DeployMode deployment_ch3_mode = DeployMode::MainPrimary;
    DeployMode deployment_ch4_mode = DeployMode::MainBackup;

    uint16_t launch_detect_altitude = 30;         // meters

    uint8_t drogue_primary_deploy_delay = 0;      // tenths of a second
    uint8_t drogue_backup_deploy_delay = 20;      // tenths of a second

    uint16_t main_primary_deploy_altitude = 130;  // meters
    uint16_t main_backup_deploy_altitude = 100;   // meters

    uint8_t deploy_signal_duration = 10;          // tenths of a second
    uint8_t lora_channel = 0;

    char device_name[device_name_length] = {0};

    // Which raw sensor axis points toward the nose (ADR-0021 Decision 6, #36).
    // Appended AFTER device_name so every existing field keeps its offset; only
    // the struct size changes.  Default Auto = pre-#36 behaviour.
    NoseAxis nose_axis = NoseAxis::Auto;
};

// Layout-sensitive: this is the payload of the RuntimeMetadataJournal, whose
// entry header carries a magic and a CRC but NO version or size field.  Any
// change to the layout therefore re-defaults the journal once on the next
// flash — including the password below.  See the migration note in ADR-0006.
struct RocketRuntimeMetadata
{
    // NOTE: an `archive_position` field led this struct until 2026-08-07.  It
    // held (last_closed_record_id + 1) — the NEXT slot to write, not the record
    // just written — and nothing ever read it.  Removed rather than renamed:
    // sitting next to last_closed_record_id under a name that reads like "the
    // record this flight went into", it was an off-by-one waiting to be picked.
    uint32_t boot_count = 0;
    uint32_t last_flight_sequence = 0;
    uint32_t last_closed_record_id = 0;
    // Connection password (plaintext; empty = open/no password).  Stored plaintext
    // so it can be shown on the UART console; the auth_tag key is derived on use
    // (PasswordKdf::DeriveKey).  Locator-only: NOT in RocketPersistentSettings, so
    // it is never carried over the air by LocatorCfgChgRequest.  Sized for the
    // 15-char UART input limit + null terminator.
    char password[16] = {0};
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable<RocketPersistentSettings>::value, "RocketPersistentSettings must be trivially copyable.");
static_assert(std::is_standard_layout<RocketPersistentSettings>::value, "RocketPersistentSettings must be standard layout.");

static_assert(std::is_trivially_copyable<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be trivially copyable.");
static_assert(std::is_standard_layout<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be standard layout.");
