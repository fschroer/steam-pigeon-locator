#pragma once
extern "C" {
#include "sys_app.h"
#include "math.h"
}

#include "Archive.hpp"
#include "Types.hpp"
#include "Navigation.hpp"
#include "PowerManagement.hpp"

//#define TEST

class FlightManager {
public:
	FlightManager(RocketNav::Navigation &nav, Archive &archive, PowerManagement &power);
	void Init();
	void UpdateFlightState();
	FlightStates GetFlightState() const {
		return flight_state_;
	}
	void SetFlight_State(FlightStates flight_state) {
		flight_state_ = flight_state;
	}
	;
	uint8_t GetDeploymentStats(uint8_t channel) const {
		switch (channel) {
		case 1:
			return deployment_ch1_stats_;
		case 2:
			return deployment_ch2_stats_;
		case 3:
			return deployment_ch3_stats_;
		case 4:
			return deployment_ch4_stats_;
		default:
			return 0xff;
		}
	}
	uint8_t GetPhysicalDeploymentStats() const {
		return physical_deployment_stats_;
	}

private:
	RocketNav::Navigation &nav_;
	Archive &archive_;
	PowerManagement &power_;

#ifdef TEST
	bool ReadFlightRecord(uint16_t archive_position);
#endif
	void CheckQueuedDeployment();
	void DeployIfClear(uint8_t channel);
	void ResetFlight();

	FlightStates flight_state_;
	uint32_t flight_time_ms = 0;
	uint8_t deployment_ch1_stats_ = 0;
	uint8_t deployment_ch2_stats_ = 0;
	uint8_t deployment_ch3_stats_ = 0;
	uint8_t deployment_ch4_stats_ = 0;
	uint8_t physical_deployment_stats_ = 0;
	bool burnout_detected_ = false;
	int noseover_time_ = 0;
	bool near_apogee_ = false;
	bool drogue_deployed_ = false;
	bool main_deployed_ = false;
	int deploy_ch1_time_ = 0;
	int deploy_ch2_time_ = 0;
	int deploy_ch3_time_ = 0;
	int deploy_ch4_time_ = 0;
	bool deploy_ch1_reset_ = false;
	bool deploy_ch2_reset_ = false;
	bool deploy_ch3_reset_ = false;
	bool deploy_ch4_reset_ = false;
	float pre_main_velocity_ = 0.0;

	bool deployment_queued_[4] = { false };

#ifdef TEST
	FlightArchive::FlightSample sample_buffer_[1024] { };
	uint32_t sample_count_ = 0;
#endif

};
