#pragma once
extern "C" {
#include "radio.h"   // where Radio_s is defined
}
#include "Communication.hpp"

class StRadioAdapter : public Communication::IRadio {
public:
    explicit StRadioAdapter(const Radio_s* r) : r_(r) {}

    void Send(const uint8_t* data, size_t len) override {
        r_->Send((uint8_t*)data, len);
    }

    void Rx(uint32_t timeout_ms) override {
        r_->Rx(timeout_ms);
    }

    void SetChannel(uint32_t freq) override {
        r_->SetChannel(freq);
    }

private:
    const Radio_s* r_;
};
