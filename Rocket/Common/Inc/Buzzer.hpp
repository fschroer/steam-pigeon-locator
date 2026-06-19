#pragma once

extern "C" {
#include <tim.h>
}

constexpr uint32_t TIMCLK = HSI48_VALUE;
constexpr uint32_t PSC    = 11;
uint8_t note_index_ = 0;
uint8_t duration_index_ = 0;

enum class Tone : uint8_t {
	Rest,
    C7, Cs7, D7, Ds7, E7, F7, Fs7, G7, Gs7, A7, As7, B7,
    C8, Cs8, D8, Ds8, E8, F8, Fs8, G8, Gs8, A8, As8, B8,
};

constexpr uint32_t computeARR(uint32_t freq) {
    return (TIMCLK / (PSC + 1) / freq) - 1;
}

struct ToneARR {
    Tone tone;
    uint32_t arr;
};

constexpr ToneARR arrTable[] = {
	{ Tone::Rest, 0 },
    { Tone::C7,  computeARR(2093) },
    { Tone::Cs7, computeARR(2217) },
    { Tone::D7,  computeARR(2349) },
    { Tone::Ds7, computeARR(2489) },
    { Tone::E7,  computeARR(2637) },
    { Tone::F7,  computeARR(2794) },
    { Tone::Fs7, computeARR(2960) },
    { Tone::G7,  computeARR(3136) },
    { Tone::Gs7, computeARR(3322) },
    { Tone::A7,  computeARR(3520) },
    { Tone::As7, computeARR(3729) },
    { Tone::B7,  computeARR(3951) },
    { Tone::C8,  computeARR(4186) },
    { Tone::Cs8, computeARR(4435) },
    { Tone::D8,  computeARR(4699) },
    { Tone::Ds8, computeARR(4978) },
    { Tone::E8,  computeARR(5274) },
    { Tone::F8,  computeARR(5588) },
    { Tone::Fs8, computeARR(5920) },
    { Tone::G8,  computeARR(6272) },
    { Tone::Gs8, computeARR(6645) },
    { Tone::A8,  computeARR(7040) },
    { Tone::As8, computeARR(7459) },
    { Tone::B8,  computeARR(7902) },
};

constexpr uint32_t getARR(Tone n) {
    for (auto &entry : arrTable)
        if (entry.tone == n)
            return entry.arr;
    return 0;
}

struct Note {
    Tone tone;
    uint8_t duration;
    uint8_t volume; // 1-3 valid
};

constexpr Note PowerOn[] = {
		{ Tone::C8, 2, 3 },
		{ Tone::Cs8, 2, 3 },
		{ Tone::D8, 2, 3 },
		{ Tone::Ds8, 2, 3 },
		{ Tone::E8, 2, 3 },
};

constexpr Note Arming[] = {
		{ Tone::C8, 2, 1 },
		{ Tone::Cs8, 2, 1 },
		{ Tone::Rest, 36, 0 },
};

constexpr Note Disarming[] = {
		{ Tone::C8, 2, 1 },
		{ Tone::Cs8, 2, 1 },
		{ Tone::Rest, 1, 0 },
		{ Tone::C8, 2, 1 },
		{ Tone::Cs8, 2, 1 },
};

constexpr Note Armed[] = {
		{ Tone::A7, 2, 1 },
		{ Tone::B7, 2, 1 },
		{ Tone::C8, 2, 1 },
		{ Tone::Rest, 34, 0 },
};

constexpr Note Landed[] = {
		{ Tone::A7, 2, 3 },
		{ Tone::B7, 2, 3 },
		{ Tone::C8, 2, 3 },
		{ Tone::Rest, 34, 0 },
};

constexpr Note AnotherOneBitesTheDust[] = {
		{ Tone::A7, 3, 3 }, // Bass intro
		{ Tone::G7, 3, 3 },

		{ Tone::E7, 12, 3 },
		{ Tone::E7, 12, 3 },
		{ Tone::E7, 12, 3 },
		{ Tone::Rest, 9, 0 },
		{ Tone::E7, 3, 3 },

		{ Tone::E7, 6, 3 },
		{ Tone::E7, 6, 3 },
		{ Tone::G7, 6, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::A7, 3, 3 },
		{ Tone::Rest, 18, 0 },
		{ Tone::A7, 3, 3 },
		{ Tone::G7, 3, 3 },

		{ Tone::E7, 12, 3 }, // Repeat
		{ Tone::E7, 12, 3 },
		{ Tone::E7, 12, 3 },
		{ Tone::Rest, 9, 0 },
		{ Tone::E7, 3, 3 },

		{ Tone::E7, 6, 3 },
		{ Tone::E7, 6, 3 },
		{ Tone::G7, 6, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::A7, 3, 3 },
		{ Tone::Rest, 24, 0 },

		{ Tone::E7, 6, 3 }, // Steve walks warily down the street with the
		{ Tone::E7, 6, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::G7, 9, 3 },
		{ Tone::E7, 6, 3 },
		{ Tone::E7, 6, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::E7, 3, 3 },

		{ Tone::B7, 6, 3 }, // brim pulled way down low
		{ Tone::B7, 6, 3 },
		{ Tone::B7, 3, 3 },
		{ Tone::B7, 6, 3 },
		{ Tone::A7, 15, 3 },
		{ Tone::G7, 12, 3 },

		{ Tone::E7, 6, 3 }, // Ain't no sound but the sound of his feet; ma-
		{ Tone::E7, 6, 3 },
		{ Tone::E7, 6, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::G7, 3, 3 },
		{ Tone::G7, 3, 3 },
		{ Tone::G7, 3, 3 },
		{ Tone::A8, 9, 3 },
		{ Tone::Rest, 3, 0 },
		{ Tone::E7, 3, 3 },

		{ Tone::B7, 6, 3 }, // -chine guns ready to go. Are you
		{ Tone::B7, 6, 3 },
		{ Tone::B7, 3, 3 },
		{ Tone::B7, 3, 3 },
		{ Tone::B7, 3, 3 },
		{ Tone::A7, 15, 3 },
		{ Tone::Rest, 6, 0 },
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },

		{ Tone::C8, 3, 3 }, // read-y, hey! Are you ready for this? Are you
		{ Tone::C8, 6, 3 },
		{ Tone::C8, 9, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::D8, 3, 3 },
		{ Tone::D8, 3, 3 },
		{ Tone::D8, 3, 3 },
		{ Tone::G7, 9, 3 },
		{ Tone::G7, 3, 3 },
		{ Tone::G7, 3, 3 },

		{ Tone::C8, 3, 3 }, // hanging on the edge of your seat?
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::D8, 15, 3 },
		{ Tone::Rest, 12, 0 },

		{ Tone::C8, 3, 3 }, // Out of the doorway the bullets rip
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::C8, 6, 3 },
		{ Tone::C8, 6, 3 },
		{ Tone::C8, 3, 3 },
		{ Tone::D8, 3, 3 },
		{ Tone::D8, 9, 3 },
		{ Tone::D8, 12, 3 },

		{ Tone::Rest, 6, 0 }, // to the sound of the beat, yeah
		{ Tone::A8, 3, 3 },
		{ Tone::A8, 3, 3 },
		{ Tone::A8, 3, 3 },
		{ Tone::A8, 3, 3 },
		{ Tone::A8, 3, 3 },
		{ Tone::D8, 9, 3 },
		{ Tone::E8, 18, 3 },

		{ Tone::Rest, 45, 0 }, // An-
		{ Tone::E7, 3, 3 },

		{ Tone::E7, 3, 3 }, // -other one bites the dust.
		{ Tone::E7, 3, 3 },
		{ Tone::E7, 6, 3 },
		{ Tone::G7, 6, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::A7, 15, 3 },
		{ Tone::Rest, 12, 0 },

		{ Tone::Rest, 45, 0 }, // An-
		{ Tone::E7, 3, 3 },

		{ Tone::E7, 3, 3 }, // -other one bites the dust.
		{ Tone::E7, 3, 3 },
		{ Tone::E7, 6, 3 },
		{ Tone::G7, 6, 3 },
		{ Tone::E7, 3, 3 },
		{ Tone::A7, 15, 3 },
		{ Tone::Rest, 12, 0 },
};

void BuzzerPlay(Tone n, uint8_t volume)
{
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);

    const uint32_t arr = getARR(n);
    __HAL_TIM_SET_AUTORELOAD(&htim16, arr);
    // Set compare to 50% duty cycle.  Without this, CCR1 retains whatever
    // CubeMX wrote at init.  When ARR < CCR1 the output is permanently HIGH
    // (solid tone) rather than a PWM square wave.
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (arr + 1) / 2);
    // Reset the counter so every note starts from the beginning of its period.
    // EGR=UG is intentionally avoided: on STM32WL, TIM16 shares the
    // TIM1_UP_TIM16_IRQn vector and writing EGR=UG sets UIF, which can fire
    // a spurious update interrupt that interferes with the first BuzzerPlay call.
    __HAL_TIM_SET_COUNTER(&htim16, 0);

    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, (GPIO_PinState)(volume & 0x02));
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, (GPIO_PinState)(volume & 0x01));
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void BuzzerStop()
{
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
}

// Reset to the beginning of a sequence.  Sets note_index_ = 0 directly so
// the very next call to BuzzerSequence / BuzzerSequenceOnce reliably starts
// at note 0 regardless of power-on state.
void BuzzerReset()
{
    note_index_ = 0;
    duration_index_ = 0;
}

// Continuously-looping sequence driver.
//
// BuzzerPlay / BuzzerStop are called ONCE per note boundary (when
// duration_index_ reaches 0), not on every tick.  The PWM timer runs
// freely between those boundaries, producing a clean, steady tone rather
// than a 20 Hz pulsed buzz.
template <size_t N>
void BuzzerSequence(const Note (&sequence)[N])
{
    if (duration_index_ == 0) {
        if (note_index_ >= N)
            note_index_ = 0;
        duration_index_ = sequence[note_index_].duration;
        if (sequence[note_index_].tone != Tone::Rest)
            BuzzerPlay(sequence[note_index_].tone, sequence[note_index_].volume);
        else
            BuzzerStop();
    }
    if (--duration_index_ == 0)
        note_index_++;
}

// Single-shot sequence driver.  Returns true on the tick after the last
// note's duration expires; resets state so BuzzerSequence can follow
// cleanly on the very next tick starting at note 0.
template <size_t N>
bool BuzzerSequenceOnce(const Note (&sequence)[N])
{
    if (duration_index_ == 0) {
        if (note_index_ >= N) {
            BuzzerReset();
            BuzzerStop();
            return true;
        }
        duration_index_ = sequence[note_index_].duration;
        if (sequence[note_index_].tone != Tone::Rest)
            BuzzerPlay(sequence[note_index_].tone, sequence[note_index_].volume);
        else
            BuzzerStop();
    }
    if (--duration_index_ == 0)
        note_index_++;
    return false;
}
