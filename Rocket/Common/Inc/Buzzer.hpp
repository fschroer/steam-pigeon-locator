#pragma once

extern "C" {
#include <tim.h>
}

constexpr uint32_t TIMCLK = 48000000;
constexpr uint32_t PSC    = 11;
uint8_t note_index_ = -1;
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
};

constexpr Note song1[] = {
		{ Tone::A7, 3 }, // Bass intro
		{ Tone::G7, 3 },

		{ Tone::E7, 12 },
		{ Tone::E7, 12 },
		{ Tone::E7, 12 },
		{ Tone::Rest, 9 },
		{ Tone::E7, 3 },

		{ Tone::E7, 6 },
		{ Tone::E7, 6 },
		{ Tone::G7, 6 },
		{ Tone::E7, 3 },
		{ Tone::A7, 3 },
		{ Tone::Rest, 18 },
		{ Tone::A7, 3 },
		{ Tone::G7, 3 },

		{ Tone::E7, 12 }, // Repeat
		{ Tone::E7, 12 },
		{ Tone::E7, 12 },
		{ Tone::Rest, 9 },
		{ Tone::E7, 3 },

		{ Tone::E7, 6 },
		{ Tone::E7, 6 },
		{ Tone::G7, 6 },
		{ Tone::E7, 3 },
		{ Tone::A7, 3 },
		{ Tone::Rest, 24 },

		{ Tone::E7, 6 }, // Steve walks warily down the street with the
		{ Tone::E7, 6 },
		{ Tone::E7, 3 },
		{ Tone::E7, 3 },
		{ Tone::E7, 3 },
		{ Tone::G7, 9 },
		{ Tone::E7, 6 },
		{ Tone::E7, 6 },
		{ Tone::E7, 3 },
		{ Tone::E7, 3 },

		{ Tone::B7, 6 }, // brim pulled way down low
		{ Tone::B7, 6 },
		{ Tone::B7, 3 },
		{ Tone::B7, 6 },
		{ Tone::A7, 15 },
		{ Tone::G7, 12 },

		{ Tone::E7, 6 }, // Ain't no sound but the sound of his feet; ma-
		{ Tone::E7, 6 },
		{ Tone::E7, 6 },
		{ Tone::E7, 3 },
		{ Tone::E7, 3 },
		{ Tone::G7, 3 },
		{ Tone::G7, 3 },
		{ Tone::G7, 3 },
		{ Tone::D8, 9 },
		{ Tone::Rest, 3 },
		{ Tone::E7, 3 },

		{ Tone::B7, 6 }, // -chine guns ready to go. Are you
		{ Tone::B7, 6 },
		{ Tone::B7, 3 },
		{ Tone::B7, 3 },
		{ Tone::B7, 3 },
		{ Tone::A7, 15 },
		{ Tone::Rest, 6 },
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },

		{ Tone::C8, 3 }, // read-y, hey! Are you ready for this? Are you
		{ Tone::C8, 6 },
		{ Tone::C8, 9 },
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },
		{ Tone::D8, 3 },
		{ Tone::D8, 3 },
		{ Tone::D8, 3 },
		{ Tone::G8, 9 },
		{ Tone::G7, 3 },
		{ Tone::G7, 3 },

		{ Tone::C8, 3 }, // hanging on the edge of your seat?
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },
		{ Tone::D8, 15 },
		{ Tone::Rest, 12 },

		{ Tone::C8, 3 }, // Out of the doorway the bullets rip
		{ Tone::C8, 3 },
		{ Tone::C8, 3 },
		{ Tone::C8, 6 },
		{ Tone::C8, 6 },
		{ Tone::C8, 3 },
		{ Tone::D8, 3 },
		{ Tone::D8, 9 },
		{ Tone::D8, 12 },

		{ Tone::Rest, 6 }, // to the sound of the beat, yeah
		{ Tone::A8, 3 },
		{ Tone::A8, 3 },
		{ Tone::A8, 3 },
		{ Tone::A8, 3 },
		{ Tone::A8, 3 },
		{ Tone::D8, 9 },
		{ Tone::E8, 18 },

		{ Tone::Rest, 45 }, // An-
		{ Tone::E7, 3 },

		{ Tone::E7, 3 }, // -other one bites the dust.
		{ Tone::E7, 3 },
		{ Tone::E7, 6 },
		{ Tone::G7, 6 },
		{ Tone::E7, 3 },
		{ Tone::A7, 15 },
		{ Tone::Rest, 12 },

		{ Tone::Rest, 45 }, // An-
		{ Tone::E7, 3 },

		{ Tone::E7, 3 }, // -other one bites the dust.
		{ Tone::E7, 3 },
		{ Tone::E7, 6 },
		{ Tone::G7, 6 },
		{ Tone::E7, 3 },
		{ Tone::A7, 15 },
		{ Tone::Rest, 12 },
};

void BuzzerPlay(Tone n)
{
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    __HAL_TIM_SET_AUTORELOAD(&htim16, getARR(n));
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void BuzzerStop()
{
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
}

void BuzzerSequence() {
	if (duration_index_ == 0) {
		note_index_++;
		if (note_index_ > sizeof(song1) / sizeof(song1[0]))
			note_index_ = 0;
		duration_index_ = song1[note_index_].duration;
	}
	if (song1[note_index_].tone != Tone::Rest) {
		if (--duration_index_ > 0)
			BuzzerPlay(song1[note_index_].tone);
		else
			BuzzerStop();
	}
	else
		duration_index_--;
}
