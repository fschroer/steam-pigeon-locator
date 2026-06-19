#pragma once
extern "C" {
#include <cstdint>

extern volatile uint32_t cm_elapsed_read_sample_raw;
extern volatile uint32_t cm_elapsed_read_sample_imu;
extern volatile uint32_t cm_elapsed_read_sample_baro;
extern volatile uint32_t cm_elapsed_read_sample_gps;
extern volatile uint32_t cm_elapsed_predict;
extern volatile uint32_t cm_elapsed_update_baro;
extern volatile uint32_t cm_elapsed_get_solution;
extern volatile uint32_t cm_elapsed_update;

}
