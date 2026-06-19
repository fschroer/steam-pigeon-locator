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

// Strapdown-vs-EKF orientation diagnostics (degrees), for validating the
// strapdown body→nav convention on the bench (ADR-0005 follow-up).  Read live in
// CubeMonitor at known attitudes (nose-up, nose-level-North, rolled-right) and
// compare the strapdown angles to the EKF's to determine the exact axis/sign map.
extern volatile float cm_strapdown_roll_deg;
extern volatile float cm_strapdown_pitch_deg;
extern volatile float cm_strapdown_yaw_deg;
extern volatile float cm_strapdown_tilt_deg;   // tilt-from-vertical (FR-P13 gate value)
extern volatile float cm_ekf_roll_deg;
extern volatile float cm_ekf_pitch_deg;
extern volatile float cm_ekf_yaw_deg;

// Strapdown↔EKF relative rotation, for deriving the fixed convention transform
// (ADR-0005).  L (left / nav-frame) = q_ekf ⊗ conj(q_strap);  R (right / body-
// frame) = conj(q_strap) ⊗ q_ekf.  Whichever quaternion stays CONSTANT as the
// locator is re-oriented (read while stationary) is the fixed offset to apply:
//   L constant → q_display = L ⊗ q_strap;   R constant → q_display = q_strap ⊗ R.
extern volatile float cm_qoff_L_w, cm_qoff_L_x, cm_qoff_L_y, cm_qoff_L_z;
extern volatile float cm_qoff_R_w, cm_qoff_R_x, cm_qoff_R_y, cm_qoff_R_z;

}
