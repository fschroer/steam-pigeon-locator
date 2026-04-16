#pragma once

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
    void addSample(float altitude_m, uint32_t timestamp_ms) {
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
