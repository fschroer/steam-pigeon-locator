#pragma once
extern "C" {
#include <cstdint>

volatile uint32_t cm_elapsed_update = 0;

volatile float cm_strapdown_roll_deg  = 0.0f;
volatile float cm_strapdown_pitch_deg = 0.0f;
volatile float cm_strapdown_yaw_deg   = 0.0f;
volatile float cm_strapdown_tilt_deg  = 0.0f;
volatile float cm_ekf_roll_deg  = 0.0f;
volatile float cm_ekf_pitch_deg = 0.0f;
volatile float cm_ekf_yaw_deg   = 0.0f;

}
