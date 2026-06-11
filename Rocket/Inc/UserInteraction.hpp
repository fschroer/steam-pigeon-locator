#pragma once

extern "C" {
#include "time.h"
#include "math.h"
#include "usart.h"
}

#include <string>
#include "FlightManager.hpp"
#include "Communication.hpp"
#include "Archive.hpp"
#include "Deployment.hpp"

#define UART_LINE_MAX_LENGTH 255
#define USER_INPUT_MAX_LENGTH 15
#define DATE_STRING_LENGTH 23
#define ALTIMETER_STRING_LENGTH 7
#define ACCELEROMETER_STRING_LENGTH 9

constexpr uint16_t uart_timeout = 5000;

enum UserInteractionState
{
  WaitingForCommand = 0,
  ConfigHome,
  EditDeployChannel1Mode,
  EditDeployChannel2Mode,
  EditDeployChannel3Mode,
  EditDeployChannel4Mode,
  EditDroguePrimaryDeployDelay,
  EditDrogueBackupDeployDelay,
  EditMainPrimaryDeployAltitude,
  EditMainBackupDeployAltitude,
  EditLoraChannel,
  EditDeviceName,
  DataHome,
  TestHome,
  TestDeploy1,
  TestDeploy2,
  TestDeploy3,
  TestDeploy4,
  DfuHome
};

class UserInteraction{
public:
  UserInteraction(FlightManager& flight,
  		Communication::Communication& comm,
			Archive& archive,
			Deployment& deploy,
			UART_HandleTypeDef& huart2);
  void ProcessChar(uint8_t uart_char, DeviceState& device_state);
  void SetUserInteractionState(UserInteractionState user_interaction_state);
  void NotifyTestComplete() { HAL_UART_Transmit(&huart2, (uint8_t*)test_complete_text_, strlen(test_complete_text_), uart_timeout); };
private:
  FlightManager& flight_;
  Communication::Communication& comm_;
  Archive& archive_;
  Deployment& deploy_;
  UART_HandleTypeDef& huart2_;

  UserInteractionState user_interaction_state_ = WaitingForCommand;
  char* uart_line_ = new char[UART_LINE_MAX_LENGTH + 1];
  char* user_input_ = new char[USER_INPUT_MAX_LENGTH + 1];
  const char* clear_screen_ = "\x1b[2J\r\0";
  const char* config_command_ = "config\0";
  const char* data_command_ = "data\0";
  const char* test_command_ = "test\0";
  const char* dfu_command_ = "dfu\0";
  const char* crlf_ = "\r\n\0";
  const char* cr_ = "\r\0";
  const char* bs_ = "\b \b\0";
  const char* config_menu_intro_ = "Rocket Locator Configuration\r\n\0";
  const char* config_save_text_ = "Saved Configuration\r\n\r\n\0";
  const char* cancel_text_ = "Cancelled\r\n\r\n\0";
  const char* deployment_ch1_mode_text_ = "0) Deployment Channel 1 Mode:\t\t\0";
  const char* deployment_ch2_mode_text_ = "1) Deployment Channel 2 Mode:\t\t\0";
  const char* deployment_ch3_mode_text_ = "2) Deployment Channel 3 Mode:\t\t\0";
  const char* deployment_ch4_mode_text_ = "3) Deployment Channel 4 Mode:\t\t\0";
  const char* drogue_primary_text_ = "Drogue Primary\0";
  const char* drogue_backup_text_ = "Drogue Backup \0";
  const char* main_primary_text_ = "Main Primary  \0";
  const char* main_backup_text_ = "Main Backup   \0";
  const char* drogue_primary_deploy_delay_text_ = "4) Drogue Primary Deploy Delay (s):\t\0";
  const char* drogue_backup_deploy_delay_text_ = "5) Drogue Backup Deploy Delay (s):\t\0";
  const char* main_primary_deploy_altitude_text_ = "6) Main Primary Deploy Altitude (m):\t\0";
  const char* main_backup_deploy_altitude_text_ = "7) Main Backup Deploy Altitude (m):\t\0";
  const char* lora_channel_text_ = "8) Lora Channel (0-63):\t\t\t\0";
  const char* device_name_text_ = "9) Device Name:\t\t\t\t\0";
  const char* num_edit_guidance_text_ = "[ = down, ] = up. Hit Enter to update, Esc to cancel.\r\n\0";
  const char* text_edit_guidance_text_ = "Type text. Hit Enter to update, Esc to cancel.\r\n\0";
  const char* deploy_mode_edit_text_ = "Edit Deploy Mode\r\n\0";
  const char* drogue_primary_deploy_delay_edit_text_ = "Edit Drogue Primary Deploy Delay (s):\r\n\0";
  const char* drogue_backup_deploy_delay_edit_text_ = "Edit Drogue Backup Deploy Delay (s):\r\n\0";
  const char* main_primary_deploy_altitude_edit_text_ = "Edit Main Primary Deploy Altitude (m):\r\n\0";
  const char* main_backup_deploy_altitude_edit_text_ = "Edit Main Backup Deploy Altitude (m):\r\n\0";
  const char* lora_channel_edit_text_ = "Edit Lora Channel (0-63):\r\n\0";
  //const char* device_name_edit_text_ = "Edit Device Name:\r\n\0";

  const char* data_menu_intro_ = "Rocket Locator Data Menu\r\n\r\n\0";
  const char* data_menu_header_ = "#  Date       Time     Apogee (m) Time to Apogee (s)\r\n\0";
  const char* data_exit_text_ = "Exiting Data Menu\r\n\r\n\0";
  const char* data_guidance_text_ = "\r\nStart terminal logging and enter a valid number to retrieve CSV output of corresponding flight\r\n";
  const char* export_header_text_ = "time_ms,raw_baro_agl_m,fused_agl_m,raw_baro_vel_mps,fused_vspeed_mps,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,lat_deg,lon_deg,flight_state,oc_start_us,oc_end_us,process_start_us,process_dur_us\0";

  const char* test_menu_intro_ = "Rocket Locator Test Menu\r\n\r\n\0";
  const char* test_deploy1_text_ = "1) Test Deployment Channel 1\r\n\0";
  const char* test_deploy2_text_ = "2) Test Deployment Channel 2\r\n\0";
  const char* test_deploy3_text_ = "3) Test Deployment Channel 3\r\n\0";
  const char* test_deploy4_text_ = "4) Test Deployment Channel 4\r\n\0";
  const char* test_exit_text_ = "Exiting Test Menu\r\n\r\n\0";
  const char* test_guidance_text_ = "\r\nSelect an option and deployment test will fire in 10 seconds\r\n";
  const char* test_complete_text_ = "Test complete, exiting test mode.\r\n\r\n\0";

  const char* dfu_intro_ = "Device Firmware Upgrade\r\n\r\n\0";
  const char* dfu_guidance_text_ = "Enter to continue, Esc to cancel\r\n\r\n\0";
  const char* dfu_warning_text_ = "Warning - device will stop working until reset by administrator\r\n\0";

  const char* max_altitude_text = "Apogee: \0";
  const char* max_altitude_sample_index_text = "Apogee time: \0";
  const char* launch_detect_altitude_text = "Launch detect: \0";
  const char* launch_detect_sample_index_text = "Launch detect time: \0";
  const char* burnout_altitude_text = "Burnout: \0";
  const char* burnout_sample_index_text = "Burnout time: \0";
  const char* nose_over_altitude_text = "Noseover: \0";
  const char* nose_over_sample_index_text = "Noseover time: \0";
  const char* drogue_primary_deploy_altitude_text = "Drogue primary: \0";
  const char* drogue_primary_deploy_sample_index_text = "Drogue primary time: \0";
  const char* drogue_backup_deploy_altitude_text = "Drogue backup: \0";
  const char* drogue_backup_deploy_sample_index_text = "Drogue backup time: \0";
  const char* drogue_velocity_threshold_altitude_text = "Drogue velocity threshold: \0";
  const char* drogue_velocity_threshold_sample_index_text = "Drogue velocity threshold time: \0";
  const char* main_primary_deploy_altitude_text = "Main primary: \0";
  const char* main_primary_deploy_sample_index_text = "Main primary time: \0";
  const char* main_backup_deploy_altitude_text = "Main backup: \0";
  const char* main_backup_deploy_sample_index_text = "Main backup time: \0";
  const char* main_velocity_threshold_altitude_text = "Main velocity threshold: \0";
  const char* main_velocity_threshold_sample_index_text = "Main velocity threshold time: \0";
  const char* landing_altitude_text = "Landing: \0";
  const char* landing_sample_index_text = "Landing time: \0";
  const char* channel1_fired_text = "Channel 1 fired: \0";
  const char* channel1_pre_fire_continuity_text = "Channel 1 pre-fire continuity: \0";
  const char* channel1_post_fire_continuity_text = "Channel 1 post-fire continuity: \0";
  const char* channel2_fired_text = "Channel 2 fired: \0";
  const char* channel2_pre_fire_continuity_text = "Channel 2 pre-fire continuity: \0";
  const char* channel2_post_fire_continuity_text = "Channel 2 post-fire continuity: \0";
  const char* channel3_fired_text = "Channel 3 fired: \0";
  const char* channel3_pre_fire_continuity_text = "Channel 3 pre-fire continuity: \0";
  const char* channel3_post_fire_continuity_text = "Channel 3 post-fire continuity: \0";
  const char* channel4_fired_text = "Channel 4 fired: \0";
  const char* channel4_pre_fire_continuity_text = "Channel 4 pre-fire continuity: \0";
  const char* channel4_post_fire_continuity_text = "Channel 4 post-fire continuity: \0";

  DeployMode deployment_ch1_mode_;
  DeployMode deployment_ch2_mode_;
  DeployMode deployment_ch3_mode_;
  DeployMode deployment_ch4_mode_;
  int drogue_primary_deploy_delay_;
  int drogue_backup_deploy_delay_;
  int main_primary_deploy_altitude_;
  int main_backup_deploy_altitude_;
  int lora_channel_;
  char device_name_[device_name_length];

  int MakeLine(char *target, const char *source1);
  int MakeLine(char *target, const char *source1, const char *source2);
  int MakeLine(char *target, const char *source1, const char *source2, const char *source3);
  int MakeCSVExportLine(char *target, const char *source1, const char *source2);
  int MakeCSVExportLine(char *target, const char *source1, const char *source2, const char *source3
      , const char *source4, const char *source5);
  const char* ToStr(uint16_t source, bool tenths);
  bool StrCmp(char *string1, const char *string2, int length);
  void DisplayConfigSettingsMenu();
  const char* DeployModeString(DeployMode deploy_mode_value);
  void AdjustDeploymentChannelMode(uint8_t uart_char, DeployMode *deploy_mode);
  void AdjustConfigNumericSetting(uint8_t uart_char, int *config_mode_setting, int max_setting_value, bool tenths);
  void AdjustConfigTextSetting(uint8_t uart_char, char *config_mode_setting);
  void DisplayDataMenu();
  void DisplayTestMenu();
  void ExportData(uint16_t archive_position);
  void ExportFlightStats(uint16_t archive_position);
  void MakeDateTime(char *target, int date, int time, int sample_index, bool time_zone_adjust, bool fractional);
  void FloatToCharArray(char *target, float source, uint8_t size, uint8_t fraction_digits);
  void DisplayDfuMenu();
  uint8_t StartBootloader();
};

extern UART_HandleTypeDef huart2;
