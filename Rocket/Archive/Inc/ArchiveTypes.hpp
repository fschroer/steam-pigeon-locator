#pragma once

#include "FlightArchiveCommon.hpp"
#include "FlightArchiveEventStats.hpp"
#include <cstdint>
#include "Types.hpp"

namespace FlightArchive
{
	enum class Statistic : uint16_t
	{
			FlightNumber = 0,
			FlightTimestampS,
			LaunchTimestampMs,
			BurnoutTimestampMs,
			ApogeeTimestampMs,
			NoseoverTimestampMs,
			DroguePrimaryDeployTimestampMs,
			DrogueBackupDeployTimestampMs,
			MainPrimaryDeployTimestampMs,
			MainBackupDeployTimestampMs,
			DrogueVelocityThresholdTimestampMs,
			MainVelocityThresholdTimestampMs,
			LandingTimestampMs,
			MaxAltitudeM,
			MaxVelocityMps,
			MaxAccelerationMps2,
			DeploymentCh1Stats,
			DeploymentCh2Stats,
			DeploymentCh3Stats,
			DeploymentCh4Stats,
			PhysicalDeploymentStats,
			mLowGLsbPerG,
			mHighGLsbPerG,
			mGyroLsbPerG,
			// EKF numerical-health counters, snapshotted twice (#38).  Two snapshots
			// rather than one because WHEN matters more than the total: non-zero at
			// launch means the filter had already failed on the pad, BEFORE the record
			// starts; zero at launch and non-zero at close means it failed in flight,
			// and the per-sample ekf_health column says on which cycle.  That single
			// distinction is what five flights could not be diagnosed without.
			//
			// Payload is EkfDiagSnapshot (4 x uint16, saturating) = exactly the 8 B a
			// StatSlot holds.  Saturation is harmless: past ~65k the answer is
			// "constantly", and the per-sample flags carry the timing.
			EkfDiagAtLaunch,
			EkfDiagAtClose,
			Count
	};

    // Saturating snapshot of InsEkf15::EkfDiag, sized to one StatSlot payload.
    struct EkfDiagSnapshot
    {
        uint16_t nonfinite_dx_drops;
        uint16_t baro_nonfinite_drops;
        uint16_t baro_gate_rejects;
        uint16_t vel_divergence_resets;
    };

#pragma pack(push, 1)

//	struct FlightSample
//	{
//			uint32_t timestamp_ms;
//			Vec3f accel;
//			Vec3f gyro;
//			float raw_baro_altitude_agl;
//			double lat_rad;
//			double lon_rad;
//	};

	struct FlightSample // new telemetry data
	{
		// == ARCHIVE_VERSION 6 layout (#38) ======================================
		// 88 B, 5 samples per 512 B chunk, 9 records.  Two structural changes paid
		// for everything added here:
		//
		//   lat/lon double -> int32 1e-7 deg   -8 B, and MORE precise, not less.
		//     The receiver reports 1e-7 deg natively, so this stores its own
		//     resolution (1.11 cm) exactly; the doubles carried rounding noise from
		//     a radian conversion that every consumer then converted back.
		//   bit-packed status -> one byte each  +3 B, deliberately spent.
		//     flight_state carried state + ARMED + EKF health, and gps_fix_sv
		//     carried fix type + satellite count.  Every reader had to know the bit
		//     layout, and adding a field meant hunting spare bits and getting masks
		//     right -- the kind of edit that silently corrupts a neighbouring field.
		//     Bytes are cheaper than that class of bug.
		//
		// record_count went 10 -> 9 to fund the rest; see the capacity note under
		// the static_assert.
		uint32_t timestamp_ms;              // (0,  +4)
		float raw_baro_altitude_agl;        // (4,  +4)
		float fused_altitude_agl;           // (8,  +4)
		float raw_baro_velocity;            // (12, +4)
		float fused_vertical_speed_mps;     // (16, +4)
		Vec3f accel;                        // (20, +12) SELECTED accel channel, gravity-inclusive
		Vec3f gyro;                         // (32, +12) body frame
		// Raw GPS position in the receiver's native units: degrees x 1e-7.
		// Range +/-214.7 deg, resolution 1.11 cm.
		int32_t lat_1e7;                    // (44, +4)
		int32_t lon_1e7;                    // (48, +4)
		// Raw GPS velocity, cm/s.  kGpsVelInvalid marks "fix carried no velocity",
		// so a genuine 0 (stationary on the pad) stays distinguishable from absent --
		// feeding a fabricated 0 to updateGpsVelocity acts as a spurious ZUPT.
		int16_t gps_vel_n_cms;              // (52, +2)
		int16_t gps_vel_e_cms;              // (54, +2)
		int16_t gps_vel_d_cms;              // (56, +2)
		uint16_t gps_h_acc_cm;              // (58, +2) horizontal accuracy, cm (0 = unknown)
		// NFR-9 strapdown attitude, packed int16.  Decode:
		//   tilt_deg    = tilt_cdeg / 100.0      (tilt-from-launch-vertical, 0..180)
		//   q_{w,x,y,z} = quat_q15[i] / 32767.0  (Y-reflected strapdown quaternion)
		int16_t tilt_cdeg;                  // (60, +2)
		int16_t quat_q15[4];                // (62, +8)
		// The accel channel NOT selected this cycle, 0.01 g/LSB (+/-327 g, covering
		// the +/-256 g high-g part).  ADR-0004 names the single-channel archive as a
		// known gap: a scale mismatch between the channels biases every accel-driven
		// state in the filter and is invisible in every flight recorded so far, so it
		// has to become measurable before the fused solution can be promoted.
		// kAccelAltInvalid marks "the other channel had no valid reading".
		int16_t accel_alt_cg[3];            // (70, +6)
		// == Status bytes -- one field each, no packing ==========================
		uint8_t flight_state;               // (76, +1) FlightStates enum value, plain
		uint8_t armed;                      // (77, +1) 0/1 (ADR-0021 Decision 4, #36)
		uint8_t ekf_health;                 // (78, +1) EkfHealthBits, 0 = healthy
		uint8_t gps_fix_type;               // (79, +1) 0-5 live fixType, 6/7 stale
		uint8_t gps_num_sv;                 // (80, +1) satellites used, saturated at 255
		uint8_t accel_source;               // (81, +1) ImuAccelSource actually used for `accel`
		uint8_t pps_status;                 // (82, +1) PpsStatus bits (#31)
		// Reserved.  Reads 0xFF on erased flash, 0 when written by this version.
		// Present so the NEXT per-sample field does not have to repeat the whole
		// version-bump / orphan-every-record cycle this change went through.
		uint8_t reserved[5];                // (83, +5) = 88
	};

#pragma pack(pop)

    static_assert(IsSerializable<FlightSample>(), "ExampleFlightSample must be serializable.");
    // Lock the on-flash layout.  88 B -> 5 samples / 512 B chunk -> 892,928 B per
    // record; 9 records = 8,036,352 B against the 8,323,072 B archive region.
    //
    // The binding constraint is SAMPLES PER CHUNK, not bytes: 512/sizeof is
    // integer division, so cost rises in steps.  Measured against this geometry
    // (8 min/record, 20 Hz, 4 KB sectors):
    //     <=  82 B -> 6 per chunk -> 10 records fit
    //     <=  90 B -> 5 per chunk ->  9 records fit
    //     <= 102 B -> 5 per chunk ->  8 records fit
    //     <= 110 B -> 4 per chunk ->  7 records fit
    // So the 5 reserved bytes above are free headroom up to 90 B; the field after
    // that costs a whole record.  Adjust record_count in Archive.hpp together with
    // this, and re-run Tests/ArchiveRoundTrip.
    static_assert(sizeof(FlightSample) == 88, "FlightSample layout changed -- re-check flash capacity AND record_count in Archive.hpp AND the app archive parser, and bump ARCHIVE_VERSION.");

    // == Decoders ============================================================
    // gps_fix_type: 0-5 = live u-blox fixType (NAV-PVT parsed within 3 s);
    //               6   = stale, NMEA on the wire (receiver reset to defaults);
    //               7   = stale otherwise (receiver silent, or no NAV-PVT).
    // Any value >= 6 means lat/lon on that row are latched, not live.
    inline constexpr bool GpsFixStale(uint8_t fix_type) { return fix_type >= 6u; }

    // ekf_health bits.  0 means the fused pair on that row is trustworthy.
    enum EkfHealthBits : uint8_t {
        kEkfVelDivergenceReset = 0x01u,  // velocity divergence guard fired
        kEkfCorrectionDropped  = 0x02u,  // injectErrorState() rejected a non-finite dx
        kEkfBaroRejected       = 0x04u,  // updateBaro() bailed (non-finite or gated)
        kEkfFusedFrozen        = 0x08u,  // fused altitude static while raw baro moved
    };

    // pps_status bits (#31) -- whether this sample's timestamp is GPS-disciplined.
    enum PpsStatus : uint8_t {
        kPpsLocked           = 0x01u,  // Pps_GetTim2TicksPerSec() != 0
        kPpsEdgeMissed       = 0x02u,  // a PPS edge was missed since the last sample
        kPpsIntervalRejected = 0x04u,  // an interval failed the sanity band
    };

    // Position conversions.  Storage is the receiver's own degrees x 1e-7; the
    // LoRa profile codec and the app both speak radians, so the boundary converts
    // rather than changing either representation.
    inline double Deg1e7ToDeg(int32_t v) { return static_cast<double>(v) * 1e-7; }
    inline double Deg1e7ToRad(int32_t v) {
        return static_cast<double>(v) * (1e-7 * 3.14159265358979323846 / 180.0);
    }
    inline int32_t RadToDeg1e7(double rad) {
        double d = rad * (180.0 / 3.14159265358979323846) * 1e7;
        d += (d >= 0.0) ? 0.5 : -0.5;
        if (d >  2147483647.0) d =  2147483647.0;
        if (d < -2147483648.0) d = -2147483648.0;
        return static_cast<int32_t>(d);
    }

    // GPS velocity is cm/s in an int16; INT16_MIN marks "no velocity fix" so a
    // genuine 0 cm/s stays distinguishable from absent.
    inline constexpr int16_t kGpsVelInvalid = -32768;
    inline constexpr bool    GpsVelValid(int16_t cms) { return cms != kGpsVelInvalid; }
    // Same convention for the non-selected accel channel.
    inline constexpr int16_t kAccelAltInvalid = -32768;
    inline constexpr bool    AccelAltValid(int16_t cg) { return cg != kAccelAltInvalid; }

    using ExampleEventStats = EventStatTraits<Statistic, 8u>;
}
