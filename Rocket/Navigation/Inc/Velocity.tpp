#pragma once
#include <cmath>
#include <cstddef>

template<size_t N>
class AltitudeRing {
public:
    AltitudeRing() : head_(0), count_(0) {}

    void push(float altitude_m, uint32_t timestamp_ms) {
        buf_[head_].alt = altitude_m;
        buf_[head_].t   = timestamp_ms;

        head_ = (head_ + 1) % N;
        if (count_ < N) count_++;
    }

    bool full() const { return count_ == N; }
    size_t size() const { return count_; }

    struct Sample { float alt; uint32_t t; };

    // Oldest sample = head_ if full, else 0
    Sample oldest() const {
        size_t idx = full() ? head_ : 0;
        return buf_[idx];
    }

    // Most recent sample = (head_ - 1 + N) % N
    Sample newest() const {
        size_t idx = (head_ + N - 1) % N;
        return buf_[idx];
    }

private:
    Sample buf_[N];
    size_t head_;
    size_t count_;
};

template<size_t N>
class VelocityEstimator {
public:
    // Maximum altitude change per 50 ms sample permitted before clamping.
    // At 200 m/s (well above any real rocket baro ascent rate) a 50 ms step
    // is 10 m.  Pyro shock transients produce 20-40 m single-step jumps that
    // exceed this limit and are clamped, preventing ±100+ m/s spikes from
    // appearing in the logged raw_baro_vel diagnostic.  Normal flight steps
    // (~5 m at 20 Hz during boost) are well within the limit.
    static constexpr float kMaxStepMps = 200.0f;

    void addSample(float altitude_m, uint32_t timestamp_ms) {
        if (ring_.size() > 0) {
            auto prev = ring_.newest();
            float dt = (timestamp_ms - prev.t) * 0.001f;
            if (dt > 0.0f) {
                float max_step = kMaxStepMps * dt;
                float delta = altitude_m - prev.alt;
                if (std::fabs(delta) > max_step)
                    altitude_m = prev.alt + (delta > 0.0f ? max_step : -max_step);
            }
        }
        ring_.push(altitude_m, timestamp_ms);
    }

    bool velocity(float& out_mps) const {
        if (ring_.size() < 2)
            return false;

        auto a = ring_.oldest();
        auto b = ring_.newest();

        float dt = (b.t - a.t) * 0.001f;   // ms → seconds
        if (dt <= 0.0f)
            return false;

        out_mps = (b.alt - a.alt) / dt;
        return true;
    }

private:
    AltitudeRing<N> ring_;
};
