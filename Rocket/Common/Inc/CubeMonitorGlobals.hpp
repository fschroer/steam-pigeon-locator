#pragma once
extern "C" {
#include <cstdint>
}

#include "Types.hpp"

extern volatile float m_agl;
extern volatile float m_velocity;
extern volatile float m_accel;
extern volatile FlightStates m_flight_state;
extern volatile DeployMode m_deploy_mode_ch1;
extern volatile DeployMode m_deploy_mode_ch2;
extern volatile float m_x, m_y, m_z;
extern volatile float m_g_force;
extern volatile uint8_t m_rocket_service_state;
extern volatile uint8_t m_radio_send;
extern volatile uint8_t m_uart1_rec;
extern volatile uint8_t m_processing_new_gga_sentence_;
extern volatile uint8_t m_processing_new_rmc_sentence_;
extern volatile uint8_t m_bad_gps_message;
