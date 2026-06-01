#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <cstring>

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
};

struct RocketRuntimeMetadata
{
    uint8_t archive_position = 0;
    uint32_t boot_count = 0;
    uint32_t last_flight_sequence = 0;
    uint32_t last_closed_record_id = 0;
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable<RocketPersistentSettings>::value, "RocketPersistentSettings must be trivially copyable.");
static_assert(std::is_standard_layout<RocketPersistentSettings>::value, "RocketPersistentSettings must be standard layout.");

static_assert(std::is_trivially_copyable<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be trivially copyable.");
static_assert(std::is_standard_layout<RocketRuntimeMetadata>::value, "RocketRuntimeMetadata must be standard layout.");
