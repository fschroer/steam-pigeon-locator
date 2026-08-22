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

    // NOT settable from anywhere today.  The app leaves the matching wire slot
    // RESERVED (Communication.cpp restores this locator's value after copying a
    // LocatorCfgChgRequest), and the console's config save does not assign it
    // either — so this default is the value in practice.  The app cannot read the
    // field back, since neither it nor deploy_signal_duration rides in
    // PreLaunchData, so it could only ever have overwritten it with a guess, which
    // is what it used to do.  Offering either in a UI means carrying it in a
    // broadcast first.
    uint16_t launch_detect_altitude = 30;         // meters — see above

    uint8_t drogue_primary_deploy_delay = 0;      // tenths of a second
    uint8_t drogue_backup_deploy_delay = 20;      // tenths of a second

    uint16_t main_primary_deploy_altitude = 130;  // meters
    uint16_t main_backup_deploy_altitude = 100;   // meters

    // Pyro firing time.  Not settable from anywhere — see launch_detect_altitude.
    uint8_t deploy_signal_duration = 10;          // tenths of a second — see above
    uint8_t lora_channel = 0;

    char device_name[device_name_length] = {0};

    // Which raw sensor axis points toward the nose (ADR-0021 Decision 6, #36).
    // Appended AFTER device_name so every existing field keeps its offset; only
    // the struct size changes.
    //
    // Defaults to X because that is the standard installation — the board's X
    // axis runs along the tube — so X is a statement of fact about the hardware,
    // not a placeholder.  Auto was the original default only to preserve pre-#36
    // behavior, and it costs the not-armed pad alert and off-pad calibration
    // outright (both gate on a stated axis), so an unconfigured locator shipped
    // with two safety features silently disabled.  A build mounted Y- or
    // Z-along-the-tube must still say so: a WRONG axis is worse than Auto, since
    // it reads an upright rocket as lying down.  See ADR-0021's 2026-08-13
    // amendment for that trade in full.
    //
    // Reaches a device ONLY through Archive::default_settings_, i.e. when the
    // settings journal has no valid entry.  A locator that has ever saved its
    // settings keeps whatever it stored; changing this line does not migrate it.
    NoseAxis nose_axis = NoseAxis::X;
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
    // UART console baud rate.  Here for the same reason as the password: it is a
    // host-link setting, and RocketPersistentSettings is the payload of the
    // over-the-air LocatorSettings message, so a value placed there could be
    // pushed to a locator by radio — muting the console of a device that may be
    // out of reach.  Always one of ConsoleBaudRates::kStandardRates; 0 means
    // "never set", which resolves to ConsoleBaudRates::kFallbackRate at load.
    uint32_t console_baud = 0;
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable<RocketPersistentSettings>::value, "RocketPersistentSettings must be trivially copyable.");
static_assert(std::is_standard_layout<RocketPersistentSettings>::value, "RocketPersistentSettings must be standard layout.");

static_assert(std::is_trivially_copyable<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be trivially copyable.");
static_assert(std::is_standard_layout<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be standard layout.");
