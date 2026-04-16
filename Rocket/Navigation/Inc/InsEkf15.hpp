#pragma once
#include <Types.hpp>

namespace RocketNav {

class InsEkf15 {
public:
    InsEkf15();

    bool initialize(const ImuSample& imu, const BaroSample* baro, const GpsSample* gps);
    void predict(const ImuSample& imu, float dt_s);
    void updateBaro(const BaroSample& baro);
    void updateGpsPosition(const GpsSample& gps);
    void updateGpsVelocity(const GpsSample& gps);

    void zeroPadReferenceAgl(float agl_m = 0.0f);
    void applyPadGyroRecalibration(const Vec3f& gyro_rps, float alpha);

    NavSolution getSolution() const { return m_sol; }
    bool isInitialized() const { return m_initialized; }

private:
    void setIdentityP(float diag);
    void symmetrizeP();
    void injectErrorState(const float dx[15]);

    NavSolution m_sol{};
    bool m_initialized = false;

    float P[15*15]{};
    float Q[15*15]{};

    double m_ref_lat_rad = 0.0;
    double m_ref_lon_rad = 0.0;
    float m_pad_altitude_msl_m = 0.0f;
    float m_pad_altitude_agl_zero_m = 0.0f;
};

}
