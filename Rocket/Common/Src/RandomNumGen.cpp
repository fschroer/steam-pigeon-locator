//#include "RandomNumGen.hpp"
//#include "stm32wlxx_hal.h"
//
//extern RNG_HandleTypeDef hrng;   // Provided by CubeMX
//
//namespace RandomNumGen {
//
//// ---- Internal PRNG state (xoshiro128++) ----
//static uint32_t s[4];
//
//// ---- Hardware RNG wrapper ----
//static uint32_t hw_random_u32()
//{
//    uint32_t val = 0;
//    if (HAL_RNG_GenerateRandomNumber(&hrng, &val) == HAL_OK)
//        return val;
//
//    // Fallback if hardware RNG fails
//    return 0xA5A5A5A5u;
//}
//
//// ---- Seed PRNG from hardware RNG ----
//void init()
//{
//    s[0] = hw_random_u32();
//    s[1] = hw_random_u32();
//    s[2] = hw_random_u32();
//    s[3] = hw_random_u32();
//
//    // Avoid all-zero state
//    if ((s[0] | s[1] | s[2] | s[3]) == 0)
//        s[0] = 1;
//}
//
//// ---- xoshiro128++ core ----
//static uint32_t rotl(const uint32_t x, int k)
//{
//    return (x << k) | (x >> (32 - k));
//}
//
//uint32_t u32()
//{
//    const uint32_t result = rotl(s[0] + s[3], 7) + s[0];
//
//    const uint32_t t = s[1] << 9;
//
//    s[2] ^= s[0];
//    s[3] ^= s[1];
//    s[1] ^= s[2];
//    s[0] ^= s[3];
//
//    s[2] ^= t;
//
//    s[3] = rotl(s[3], 11);
//
//    return result;
//}
//
//uint16_t u16()
//{
//    return static_cast<uint16_t>(u32() >> 16);
//}
//
//float unit_float()
//{
//    return (u32() >> 8) * (1.0f / 16777216.0f); // 24-bit mantissa
//}
//
//} // namespace RNG
