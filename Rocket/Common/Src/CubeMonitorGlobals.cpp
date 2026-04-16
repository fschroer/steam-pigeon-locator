#include "CubeMonitorGlobals.hpp"

volatile float m_agl = 0.0;
volatile float m_velocity = 0.0;
volatile float m_accel = 0.0;
volatile FlightStates m_flight_state = FlightStates::WaitingLaunch;
volatile DeployMode m_deploy_mode_ch1 = DeployMode::DroguePrimary;
volatile DeployMode m_deploy_mode_ch2 = DeployMode::MainPrimary;
volatile DeployMode m_deploy_mode_ch3 = DeployMode::DroguePrimary;
volatile DeployMode m_deploy_mode_ch4 = DeployMode::MainPrimary;
volatile float m_x = 0.0, m_y = 0.0, m_z = 0.0;
volatile float m_g_force = 0.0;
volatile uint8_t m_rocket_service_state = 0;
volatile uint8_t m_radio_send = 0;
volatile uint8_t m_uart1_rec = 0;
volatile uint8_t m_processing_new_gga_sentence_ = 0;
volatile uint8_t m_processing_new_rmc_sentence_ = 0;
volatile uint8_t m_bad_gps_message = 0;
