extern "C" {
#include "usart.h"
}

#include <UserInteraction.hpp>
#include "CubeMonitorGlobals.hpp"
#include "Constants.hpp"
#include "StaticString.hpp"
#include "StaticStringWriter.hpp"
#include "Format.hpp"
#include "Units.hpp"

constexpr uint8_t century = 100;
constexpr uint16_t max_drogue_primary_deploy_delay = 20;
constexpr uint16_t max_drogue_backup_deploy_delay = 40;
constexpr uint16_t max_main_primary_deploy_altitude = 400;
constexpr uint16_t max_main_backup_deploy_altitude = 400;
constexpr uint16_t max_lora_channel = 63;

UserInteraction::UserInteraction(FlightManager &flight, Communication::Communication &comm, Archive &archive,
		Deployment &deploy, UART_HandleTypeDef &huart2) :
		flight_(flight), comm_(comm), archive_(archive), deploy_(deploy), huart2_(huart2) {
}

void UserInteraction::ProcessChar(uint8_t uart_char, DeviceState &device_state) {
	RocketPersistentSettings &locator_settings = archive_.GetLocatorSettings();
	static int char_pos = 0;
	int uart_line_len = 0;
	switch (user_interaction_state_) {
	case UserInteractionState::WaitingForCommand:
		if (((uart_char >= 'A' && uart_char <= 'Z') || (uart_char >= 'a' && uart_char <= 'z'))
				&& char_pos < USER_INPUT_MAX_LENGTH) {
			HAL_UART_Transmit(&huart2_, &uart_char, 1, uart_timeout);
			user_input_[char_pos++] = uart_char;
		} else
			switch (uart_char) {
			case 13: // Enter key
				if (StrCmp(user_input_, config_command_, char_pos)) {
					device_state = DeviceState::Config;
					user_interaction_state_ = UserInteractionState::ConfigHome;
					deployment_ch1_mode_ = locator_settings.deployment_ch1_mode;
					deployment_ch2_mode_ = locator_settings.deployment_ch2_mode;
					drogue_primary_deploy_delay_ = locator_settings.drogue_primary_deploy_delay;
					drogue_backup_deploy_delay_ = locator_settings.drogue_backup_deploy_delay;
					main_primary_deploy_altitude_ = locator_settings.main_primary_deploy_altitude;
					main_backup_deploy_altitude_ = locator_settings.main_backup_deploy_altitude;
					lora_channel_ = locator_settings.lora_channel;
					std::memcpy(device_name_, locator_settings.device_name, device_name_length);
					DisplayConfigSettingsMenu();
				} else if (StrCmp(user_input_, data_command_, char_pos)) {
					device_state = DeviceState::Config;
					user_interaction_state_ = UserInteractionState::DataHome;
					DisplayDataMenu();
				} else if (StrCmp(user_input_, test_command_, char_pos)) {
					device_state = DeviceState::Config;
					user_interaction_state_ = UserInteractionState::TestHome;
					DisplayTestMenu();
				} else if (StrCmp(user_input_, dfu_command_, char_pos)) {
					device_state = DeviceState::Config;
					user_interaction_state_ = UserInteractionState::DfuHome;
					DisplayDfuMenu();
				}
				char_pos = 0;
				user_input_[0] = 0;
				break;
			case 8: // Backspace
				HAL_UART_Transmit(&huart2_, &uart_char, 1, uart_timeout);
				user_input_[--char_pos] = 0;
				break;
			}
		break;
	case UserInteractionState::ConfigHome:
		switch (uart_char) {
		case 13: // Enter key
			locator_settings.deployment_ch1_mode = deployment_ch1_mode_;
			locator_settings.deployment_ch2_mode = deployment_ch2_mode_;
			locator_settings.deployment_ch3_mode = deployment_ch3_mode_;
			locator_settings.deployment_ch4_mode = deployment_ch4_mode_;
			locator_settings.drogue_primary_deploy_delay = drogue_primary_deploy_delay_;
			locator_settings.drogue_backup_deploy_delay = drogue_backup_deploy_delay_;
			locator_settings.main_primary_deploy_altitude = main_primary_deploy_altitude_;
			locator_settings.main_backup_deploy_altitude = main_backup_deploy_altitude_;
			locator_settings.lora_channel = lora_channel_;
			std::memcpy(locator_settings.device_name, device_name_, device_name_length);
			archive_.SaveLocatorSettings(locator_settings);
			comm_.SetChannel(lora_channel_);
			device_state = DeviceState::Disarmed;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			uart_line_len = MakeLine(uart_line_, config_save_text_);
			break;
		case 27: // Esc key
			device_state = DeviceState::Disarmed;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			uart_line_len = MakeLine(uart_line_, cancel_text_);
			break;
		case 48: // 0 = Edit deployment channel 1 mode
			user_interaction_state_ = UserInteractionState::EditDeployChannel1Mode;
			uart_line_len = MakeLine(uart_line_, deploy_mode_edit_text_, num_edit_guidance_text_,
					DeployModeString(deployment_ch1_mode_));
			break;
		case 49: // 1 = Edit deployment channel 2 mode
			user_interaction_state_ = UserInteractionState::EditDeployChannel2Mode;
			uart_line_len = MakeLine(uart_line_, deploy_mode_edit_text_, num_edit_guidance_text_,
					DeployModeString(deployment_ch2_mode_));
			break;
		case 50: // 2 = Edit deployment channel 3 mode
			user_interaction_state_ = UserInteractionState::EditDeployChannel3Mode;
			uart_line_len = MakeLine(uart_line_, deploy_mode_edit_text_, num_edit_guidance_text_,
					DeployModeString(deployment_ch3_mode_));
			break;
		case 51: // 3 = Edit deployment channel 4 mode
			user_interaction_state_ = UserInteractionState::EditDeployChannel3Mode;
			uart_line_len = MakeLine(uart_line_, deploy_mode_edit_text_, num_edit_guidance_text_,
					DeployModeString(deployment_ch4_mode_));
			break;
		case 52: // 4 = Edit drogue primary deploy delay
			user_interaction_state_ = UserInteractionState::EditDroguePrimaryDeployDelay;
			uart_line_len = MakeLine(uart_line_, drogue_primary_deploy_delay_edit_text_, num_edit_guidance_text_,
					ToStr(drogue_primary_deploy_delay_, true));
			break;
		case 53: // 5 = Edit drogue backup deploy delay
			user_interaction_state_ = UserInteractionState::EditDrogueBackupDeployDelay;
			uart_line_len = MakeLine(uart_line_, drogue_backup_deploy_delay_edit_text_, num_edit_guidance_text_,
					ToStr(drogue_backup_deploy_delay_, true));
			break;
		case 54: // 6 = Edit main primary deploy altitude
			user_interaction_state_ = UserInteractionState::EditMainPrimaryDeployAltitude;
			uart_line_len = MakeLine(uart_line_, main_primary_deploy_altitude_edit_text_, num_edit_guidance_text_,
					ToStr(main_primary_deploy_altitude_, false));
			break;
		case 55: // 7 = Edit main backup deploy altitude
			user_interaction_state_ = UserInteractionState::EditMainBackupDeployAltitude;
			uart_line_len = MakeLine(uart_line_, main_backup_deploy_altitude_edit_text_, num_edit_guidance_text_,
					ToStr(main_backup_deploy_altitude_, false));
			break;
		case 56: // 8 = Edit LoRa channel
			user_interaction_state_ = UserInteractionState::EditLoraChannel;
			uart_line_len = MakeLine(uart_line_, lora_channel_edit_text_, num_edit_guidance_text_,
					ToStr(lora_channel_, false));
			break;
		case 57: // 9 = Edit device name
			user_interaction_state_ = UserInteractionState::EditDeviceName;
			uart_line_len = MakeLine(uart_line_, text_edit_guidance_text_);
			break;
		}
		HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
		break;
	case UserInteractionState::EditDeployChannel1Mode:
		AdjustDeploymentChannelMode(uart_char, &deployment_ch1_mode_);
		break;
	case UserInteractionState::EditDeployChannel2Mode:
		AdjustDeploymentChannelMode(uart_char, &deployment_ch2_mode_);
		break;
	case UserInteractionState::EditDeployChannel3Mode:
		AdjustDeploymentChannelMode(uart_char, &deployment_ch3_mode_);
		break;
	case UserInteractionState::EditDeployChannel4Mode:
		AdjustDeploymentChannelMode(uart_char, &deployment_ch4_mode_);
		break;
	case UserInteractionState::EditDroguePrimaryDeployDelay:
		AdjustConfigNumericSetting(uart_char, &drogue_primary_deploy_delay_, max_drogue_primary_deploy_delay, true);
		break;
	case UserInteractionState::EditDrogueBackupDeployDelay:
		AdjustConfigNumericSetting(uart_char, &drogue_backup_deploy_delay_, max_drogue_backup_deploy_delay, true);
		break;
	case UserInteractionState::EditMainPrimaryDeployAltitude:
		AdjustConfigNumericSetting(uart_char, &main_primary_deploy_altitude_, max_main_primary_deploy_altitude, false);
		break;
	case UserInteractionState::EditMainBackupDeployAltitude:
		AdjustConfigNumericSetting(uart_char, &main_backup_deploy_altitude_, max_main_backup_deploy_altitude, false);
		break;
	case UserInteractionState::EditLoraChannel:
		AdjustConfigNumericSetting(uart_char, &lora_channel_, max_lora_channel, false);
		break;
	case UserInteractionState::EditDeviceName:
		AdjustConfigTextSetting(uart_char, device_name_);
		break;
	case UserInteractionState::DataHome:
		if (uart_char >= '0' && uart_char <= '9') {
			if (!archive_.IsInitialized())
				archive_.InitializeArchive();
			ExportFlightStats((uint16_t) uart_char - '0');
			ExportData((uint16_t) uart_char - '0');
		} else if (uart_char == 27) { // Esc key
			device_state = DeviceState::Disarmed;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			uart_line_len = MakeLine(uart_line_, cancel_text_);
			HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
		}
		break;
	case UserInteractionState::TestHome:
		if (uart_char == '1') {
			deploy_.ResetTestDeployment();
			deploy_.SetActiveDeploymentChannel(1);
			device_state = DeviceState::Test;
			user_interaction_state_ = UserInteractionState::TestDeploy1;
		} else if (uart_char == '2') {
			deploy_.ResetTestDeployment();
			deploy_.SetActiveDeploymentChannel(2);
			device_state = DeviceState::Test;
			user_interaction_state_ = UserInteractionState::TestDeploy2;
		} else if (uart_char == '3') {
			deploy_.ResetTestDeployment();
			deploy_.SetActiveDeploymentChannel(3);
			device_state = DeviceState::Test;
			user_interaction_state_ = UserInteractionState::TestDeploy3;
		} else if (uart_char == '4') {
			deploy_.ResetTestDeployment();
			deploy_.SetActiveDeploymentChannel(4);
			device_state = DeviceState::Test;
			user_interaction_state_ = UserInteractionState::TestDeploy4;
		} else if (uart_char == 27) { // Esc key
			deploy_.ResetTestDeployment();
			device_state = DeviceState::Disarmed;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			uart_line_len = MakeLine(uart_line_, cancel_text_);
			HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
		}
		break;
	case UserInteractionState::TestDeploy1:
	case UserInteractionState::TestDeploy2:
	case UserInteractionState::TestDeploy3:
	case UserInteractionState::TestDeploy4:
		if (uart_char == 27) { // Esc key
			device_state = DeviceState::Disarmed;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			uart_line_len = MakeLine(uart_line_, cancel_text_);
			HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
		}
		break;
	case UserInteractionState::DfuHome:
		if (uart_char == 13) // Enter key
			StartBootloader();
		else if (uart_char == 27) { // Esc key
			device_state = DeviceState::Disarmed;
			user_interaction_state_ = UserInteractionState::WaitingForCommand;
			uart_line_len = MakeLine(uart_line_, cancel_text_);
			HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
		}
		break;
	}
}

int UserInteraction::MakeLine(char *target, const char *source1) {
	int i = 0;
	for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
		target[i] = source1[i];
	target[i] = 0;
	return i;
}

int UserInteraction::MakeLine(char *target, const char *source1, const char *source2) {
	int i = 0;
	for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
		target[i] = source1[i];
	int j = 0;
	for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source2[j];
	target[i] = 0;
	return i;
}

int UserInteraction::MakeLine(char *target, const char *source1, const char *source2, const char *source3) {
	int i = 0;
	for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
		target[i] = source1[i];
	int j = 0;
	for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source2[j];
	j = 0;
	for (; source3[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source3[j];
	target[i] = 0;
	return i;
}

int UserInteraction::MakeCSVExportLine(char *target, const char *source1, const char *source2) {
	int i = 0;
	for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
		target[i] = source1[i];
	target[i++] = ',';
	int j = 0;
	for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source2[j];
	target[i++] = '\r';
	target[i++] = '\n';
	target[i] = 0;
	return i;
}

int UserInteraction::MakeCSVExportLine(char *target, const char *source1, const char *source2, const char *source3,
		const char *source4, const char *source5) {
	int i = 0;
	for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
		target[i] = source1[i];
	target[i++] = ',';
	int j = 0;
	for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source2[j];
	target[i++] = ',';
	j = 0;
	for (; source3[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source3[j];
	target[i] = 0;
	target[i++] = ',';
	j = 0;
	for (; source4[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source4[j];
	target[i] = 0;
	target[i++] = ',';
	j = 0;
	for (; source5[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
		target[i] = source5[j];
	target[i++] = '\r';
	target[i++] = '\n';
	target[i] = 0;
	return i;
}

const char* UserInteraction::ToStr(uint16_t source, bool tenths) {
	uint16_t l_source = source;
	int source_len = 0;
	if (source > 0)
		source_len = log10(source) + 1;
	else
		source_len = 1;
	char *target = new char[source_len + (tenths ? 1 : 0) + (tenths && source < 10 ? 1 : 0) + 1];
	int j = 0;
	if (tenths && source < 10)
		target[j++] = '0';
	for (int i = pow(10, source_len - 1); i > 0; i /= 10) {
		if (tenths && i == 1)
			target[j++] = '.';
		int digit = l_source / i;
		l_source -= digit * i;
		target[j++] = digit + '0';
	}
	target[j] = 0;
	return (const char*) target;
}

bool UserInteraction::StrCmp(char *string1, const char *string2, int length) {
	int i = 0;
	while (string1[i] == string2[i]) {
		if (++i == length)
			return true;
	}
	return false;
}

void UserInteraction::DisplayConfigSettingsMenu() {
	int uart_line_len = 0;
	uart_line_len = MakeLine(uart_line_, clear_screen_, config_menu_intro_, crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, deployment_ch1_mode_text_, DeployModeString(deployment_ch1_mode_), crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, deployment_ch2_mode_text_, DeployModeString(deployment_ch2_mode_), crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, deployment_ch3_mode_text_, DeployModeString(deployment_ch3_mode_), crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, deployment_ch4_mode_text_, DeployModeString(deployment_ch4_mode_), crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, drogue_primary_deploy_delay_text_, ToStr(drogue_primary_deploy_delay_, true),
			crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, drogue_backup_deploy_delay_text_, ToStr(drogue_backup_deploy_delay_, true),
			crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, main_primary_deploy_altitude_text_,
			ToStr(main_primary_deploy_altitude_, false), crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, main_backup_deploy_altitude_text_, ToStr(main_backup_deploy_altitude_, false),
			crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, lora_channel_text_, ToStr(lora_channel_, false), crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, device_name_text_, device_name_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	uart_line_len = MakeLine(uart_line_, crlf_, crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
}

void UserInteraction::DisplayDataMenu() {
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	char archive_position[] = { '0', ')', ' ', 0 };
	export_line.WriteMany(clear_screen_, data_menu_intro_, data_menu_header_);
	for (uint8_t i = 0; i < record_count; i++) {
		bool valid_record = false;
		uint32_t flight_timestamp = 0;
		archive_.ReadEvent(i, FlightArchive::ExampleStatId::FlightTimestampS, flight_timestamp, valid_record);
//    if (valid_record){
		archive_position[0] = i + '0';
		char flight_date_time[20] { 0 };
		FormatUnixUtc(flight_date_time, flight_timestamp);
		float apogee = 0.0f;
		archive_.ReadEvent(i, FlightArchive::ExampleStatId::MaxAltitudeM, apogee, valid_record);
		uint32_t apogee_timestamp = 0;
		archive_.ReadEvent(i, FlightArchive::ExampleStatId::ApogeeTimestampMs, apogee_timestamp, valid_record);
		export_line.WriteMany(archive_position, flight_date_time, Fmt(apogee, 11), Fmt(apogee_timestamp, 19), crlf_);
//    }
	}
	export_line.WriteMany(data_guidance_text_, crlf_);
}

void UserInteraction::DisplayTestMenu() {
	int uart_line_len = 0;
	uart_line_len = MakeLine(uart_line_, clear_screen_, test_menu_intro_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	HAL_UART_Transmit(&huart2_, (uint8_t*) test_deploy1_text_, strlen(test_deploy1_text_), uart_timeout);
	HAL_UART_Transmit(&huart2_, (uint8_t*) test_deploy2_text_, strlen(test_deploy2_text_), uart_timeout);
	uart_line_len = MakeLine(uart_line_, test_guidance_text_, crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
}

const char* UserInteraction::DeployModeString(DeployMode deploy_mode_value) {
	switch (deploy_mode_value) {
	case DeployMode::DroguePrimary:
		return drogue_primary_text_;
		break;
	case DeployMode::DrogueBackup:
		return drogue_backup_text_;
		break;
	case DeployMode::MainPrimary:
		return main_primary_text_;
		break;
	case DeployMode::MainBackup:
		return main_backup_text_;
		break;
	default:
		break;
	}
	return "\0";
}

void UserInteraction::AdjustDeploymentChannelMode(uint8_t uart_char, DeployMode *deploy_mode) {
	int uart_line_len = 0;
	switch (uart_char) {
	case 13: // Enter key
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	case 27: // Esc key
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	case 91: // [ = decrease value
		switch (*deploy_mode) {
		case DeployMode::DroguePrimary:
			*deploy_mode = DeployMode::MainBackup;
			break;
		case DeployMode::DrogueBackup:
			*deploy_mode = DeployMode::DroguePrimary;
			break;
		case DeployMode::MainPrimary:
			*deploy_mode = DeployMode::DrogueBackup;
			break;
		case DeployMode::MainBackup:
			*deploy_mode = DeployMode::MainPrimary;
			break;
		default:
			break;
		}
		uart_line_len = MakeLine(uart_line_, cr_, DeployModeString(*deploy_mode));
		HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
		break;
	case 93: // [ = increase value
		switch (*deploy_mode) {
		case DeployMode::DroguePrimary:
			*deploy_mode = DeployMode::DrogueBackup;
			break;
		case DeployMode::DrogueBackup:
			*deploy_mode = DeployMode::MainPrimary;
			break;
		case DeployMode::MainPrimary:
			*deploy_mode = DeployMode::MainBackup;
			break;
		case DeployMode::MainBackup:
			*deploy_mode = DeployMode::DroguePrimary;
			break;
		default:
			break;
		}
		uart_line_len = MakeLine(uart_line_, cr_, DeployModeString(*deploy_mode));
		HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
		break;
	}
}

void UserInteraction::AdjustConfigNumericSetting(uint8_t uart_char, int *config_mode_setting, int max_setting_value,
		bool tenths) {
	uint16_t uart_line_len = 0;
	switch (uart_char) {
	case 13: // Enter key
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	case 27: // Esc key
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
		break;
	case 91: // [ = decrease value
		if (*config_mode_setting > 0)
			(*config_mode_setting)--;
		break;
	case 93: // ] = increase value
		if (*config_mode_setting < max_setting_value)
			(*config_mode_setting)++;
		break;
	}
	if (uart_char == 91 || uart_char == 93) {
		uart_line_len = MakeLine(uart_line_, cr_, ToStr(*config_mode_setting, tenths));
		HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
	}
}

void UserInteraction::AdjustConfigTextSetting(uint8_t uart_char, char *config_mode_setting) {
	static uint16_t char_pos = 0;
	if (uart_char == 13 || uart_char == 27) {
		if (uart_char == 13) {
			uint16_t i = 0;
			for (; i < char_pos; i++)
				config_mode_setting[i] = user_input_[i];
			for (; i < device_name_length; i++)
				config_mode_setting[i] = 0;
		}
		char_pos = 0;
		for (uint8_t i = 0; i < device_name_length; i++)
			user_input_[i] = 0;
		user_interaction_state_ = UserInteractionState::ConfigHome;
		DisplayConfigSettingsMenu();
	} else if (uart_char == 8 && char_pos > 0) {
		HAL_UART_Transmit(&huart2_, (uint8_t*) bs_, 3, uart_timeout);
		user_input_[--char_pos] = 0;
	} else if (uart_char >= ' ' && uart_char <= '~' && char_pos < device_name_length - 1) {
		HAL_UART_Transmit(&huart2_, &uart_char, 1, uart_timeout);
		user_input_[char_pos++] = uart_char;
	}
}

void UserInteraction::ExportData(uint16_t archive_position) {
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	export_line.WriteMany(clear_screen_, export_header_text_, crlf_);

	uint32_t sample_count_out = 0;
	bool success = archive_.GetFlightSampleCount(archive_position, sample_count_out);
	if (success) {
		FlightArchive::FlightSample sample_buffer[64];
//		FlightArchive::FlightSample sample_buffer[sample_count_out] { };
		uint32_t got = 0u;
		uint32_t start = 0u;

		while (true) {
		    if (!archive_.ReadFlightDataRange(archive_position, start, sample_buffer, 64u, got)) {break;}
		    if (got == 0u) {break;}
		    for (uint32_t i = 0u; i < got; ++i) {
//				export_line.WriteMany(sample_buffer[i].timestamp_ms, ",", Fmt(sample_buffer[i].fused_altitude_agl, 0, 1), ",", // new telemetry data. Add other new elements when uncommenting
				export_line.WriteMany(sample_buffer[i].timestamp_ms, ",", Fmt(sample_buffer[i].raw_baro_altitude_agl, 0, 1), ",",
						Fmt(sample_buffer[i].accel.x / G0_F, 0, 1), ",", Fmt(sample_buffer[i].accel.y / G0_F, 0, 1),
						",", Fmt(sample_buffer[i].accel.z / G0_F, 0, 1), ",",
						Fmt(sample_buffer[i].gyro.x * RAD2DEG, 0, 1), ",", Fmt(sample_buffer[i].gyro.y * RAD2DEG, 0, 1),
						",", Fmt(sample_buffer[i].gyro.z * RAD2DEG, 0, 1), ",",
						Fmt(sample_buffer[i].lat_rad * RAD2DEG, 0, 7), ",",
						Fmt(sample_buffer[i].lon_rad * RAD2DEG, 0, 7), crlf_);
		    }
		    start += got;
		}
//		uint32_t samples_read_out = 0;
//		if (archive_.ReadFlightData(archive_position, sample_buffer, sample_count_out, samples_read_out)) {
//			for (uint32_t i = 0; i < samples_read_out; i++) {
//			}
//		}
	}
}

void UserInteraction::ExportFlightStats(uint16_t archive_position) { //Export flight statistics
	StaticStringWriter<UART_LINE_MAX_LENGTH> export_line(&huart2_);
	uint32_t time_ms = 0;
	bool present_out;
	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::FlightTimestampS, time_ms, present_out);
	char flight_date_time[20] { };
	FormatUnixUtc(flight_date_time, time_ms);
	export_line.WriteMany("Flight time: ", flight_date_time, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::LaunchTimestampMs, time_ms, present_out);
	export_line.WriteMany(launch_detect_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::BurnoutTimestampMs, time_ms, present_out);
	export_line.WriteMany(burnout_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::ApogeeTimestampMs, time_ms, present_out);
	export_line.WriteMany(max_altitude_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::NoseoverTimestampMs, time_ms, present_out);
	export_line.WriteMany(nose_over_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::DroguePrimaryDeployTimestampMs, time_ms,
			present_out);
	export_line.WriteMany(drogue_primary_deploy_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::DrogueBackupDeployTimestampMs, time_ms,
			present_out);
	export_line.WriteMany(drogue_backup_deploy_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::DrogueVelocityThresholdTimestampMs, time_ms,
			present_out);
	export_line.WriteMany("Drogue velocity threshold time: ", time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::MainPrimaryDeployTimestampMs, time_ms,
			present_out);
	export_line.WriteMany(main_primary_deploy_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::MainBackupDeployTimestampMs, time_ms,
			present_out);
	export_line.WriteMany(main_backup_deploy_sample_index_text, time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::MainVelocityThresholdTimestampMs, time_ms,
			present_out);
	export_line.WriteMany("Main velocity threshold time: ", time_ms, crlf_);

	archive_.ReadEvent(archive_position, FlightArchive::ExampleStatId::LandingTimestampMs, time_ms, present_out);
	export_line.WriteMany(landing_sample_index_text, time_ms, crlf_);

//  uart_line_len = MakeLine(export_line, channel1_fired_text, (flight_stats_msg_.deployment_state & (1 << BIT_SHIFT_CHANNEL_1_FIRED)) == 0 ? "No" : "Yes", crlf_);
//  uart_line_len = MakeLine(export_line, channel1_pre_fire_continuity_text, (flight_stats_msg_.deployment_state & (1 << BIT_SHIFT_CHANNEL_1_PRE_FIRE_CONTINUITY)) == 0 ? "No" : "Yes", crlf_);
//  uart_line_len = MakeLine(export_line, channel1_post_fire_continuity_text, (flight_stats_msg_.deployment_state & (1 << BIT_SHIFT_CHANNEL_1_POST_FIRE_CONTINUITY)) == 0 ? "No" : "Yes", crlf_);
//  uart_line_len = MakeLine(export_line, channel2_fired_text, (flight_stats_msg_.deployment_state & (1 << BIT_SHIFT_CHANNEL_2_FIRED)) == 0 ? "No" : "Yes", crlf_);
//  uart_line_len = MakeLine(export_line, channel2_pre_fire_continuity_text, (flight_stats_msg_.deployment_state & (1 << BIT_SHIFT_CHANNEL_2_PRE_FIRE_CONTINUITY)) == 0 ? "No" : "Yes", crlf_);
//  uart_line_len = MakeLine(export_line, channel2_post_fire_continuity_text, (flight_stats_msg_.deployment_state & (1 << BIT_SHIFT_CHANNEL_2_POST_FIRE_CONTINUITY)) == 0 ? "No" : "Yes", crlf_);
}

void UserInteraction::DisplayDfuMenu() {
	int uart_line_len = MakeLine(uart_line_, clear_screen_, uart_line_, dfu_intro_);
	uart_line_len += MakeLine(uart_line_ + uart_line_len, dfu_guidance_text_, dfu_warning_text_, crlf_);
	HAL_UART_Transmit(&huart2_, (uint8_t*) uart_line_, uart_line_len, uart_timeout);
}

uint8_t UserInteraction::StartBootloader() {
	FLASH_OBProgramInitTypeDef ob_cfg;
	HAL_FLASHEx_OBGetConfig(&ob_cfg);
	ob_cfg.OptionType = OPTIONBYTE_USER;
	ob_cfg.UserType = OB_USER_nBOOT0 | OB_USER_nBOOT1 | OB_USER_nSWBOOT0;
	ob_cfg.UserConfig = OB_BOOT0_RESET | OB_BOOT1_SET | OB_BOOT0_FROM_OB;

	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBProgram(&ob_cfg);
//    HAL_FLASH_OB_Launch();
//    HAL_FLASH_OB_Lock();
//    HAL_FLASH_Lock();
	return 0;
}

void UserInteraction::SetUserInteractionState(UserInteractionState user_interaction_state) {
	user_interaction_state_ = user_interaction_state;
}
