#ifndef LITTLE_ENDIAN_DECODING_H_
#define LITTLE_ENDIAN_DECODING_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
 extern "C" {
#endif

#if !defined(__BYTE_ORDER__)
    #define __ORDER_LITTLE_ENDIAN__ 1
    #define __ORDER_BIG_ENDIAN__ 2
    // Default value
    #define __BYTE_ORDER__ __ORDER_LITTLE_ENDIAN__
#endif

// Byte offsets for decoding little endian values
// to C standard types
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    // 16-bit values
    const size_t W16_OFFSET_BYTE_0 = 0;
    const size_t W16_OFFSET_BYTE_1 = 8;

    // 32-bit values
    const size_t W32_OFFSET_BYTE_0 = 0;
    const size_t W32_OFFSET_BYTE_1 = 8;
    const size_t W32_OFFSET_BYTE_2 = 16;
    const size_t W32_OFFSET_BYTE_3 = 24;

    // 64-bit values
    const size_t W64_OFFSET_BYTE_0 = 0;
    const size_t W64_OFFSET_BYTE_1 = 8;
    const size_t W64_OFFSET_BYTE_2 = 16;
    const size_t W64_OFFSET_BYTE_3 = 24;
    const size_t W64_OFFSET_BYTE_4 = 32;
    const size_t W64_OFFSET_BYTE_5 = 40;
    const size_t W64_OFFSET_BYTE_6 = 48;
    const size_t W64_OFFSET_BYTE_7 = 56;

#else
    // 16-bit values
    const size_t W16_OFFSET_BYTE_0 = 8;
    const size_t W16_OFFSET_BYTE_1 = 0;

    // 32-bit values
    const size_t W32_OFFSET_BYTE_0 = 24;
    const size_t W32_OFFSET_BYTE_1 = 16;
    const size_t W32_OFFSET_BYTE_2 = 8;
    const size_t W32_OFFSET_BYTE_3 = 0;

    // 64-bit values
    const size_t W64_OFFSET_BYTE_0 = 56;
    const size_t W64_OFFSET_BYTE_1 = 48;
    const size_t W64_OFFSET_BYTE_2 = 40;
    const size_t W64_OFFSET_BYTE_3 = 32;
    const size_t W64_OFFSET_BYTE_4 = 24;
    const size_t W64_OFFSET_BYTE_5 = 16;
    const size_t W64_OFFSET_BYTE_6 = 8;
    const size_t W64_OFFSET_BYTE_7 = 0;
#endif

/* ------------------------------------ - ----------------------------------- */

inline uint16_t get_16_bit_representation_(void* p_serialized_value)
{
    const uint8_t* it_byte = (uint8_t*) p_serialized_value;

    uint16_t x = 0;
    x |= ((uint16_t)it_byte[0]) << W16_OFFSET_BYTE_0;
    x |= ((uint16_t)it_byte[1]) << W16_OFFSET_BYTE_1;

    return x;
}  

inline uint32_t get_32_bit_representation_(void* p_serialized_value)
{
    const uint8_t* it_byte = (uint8_t*) p_serialized_value;

    uint32_t x = 0;
    x |= ((uint32_t)it_byte[0]) << W32_OFFSET_BYTE_0;
    x |= ((uint32_t)it_byte[1]) << W32_OFFSET_BYTE_1;
    x |= ((uint32_t)it_byte[2]) << W32_OFFSET_BYTE_2;
    x |= ((uint32_t)it_byte[3]) << W32_OFFSET_BYTE_3;

    return x;
}

inline uint64_t get_64_bit_representation_(void* p_serialized_value)
{
    const uint8_t* it_byte = (uint8_t*) p_serialized_value;

    uint64_t x = 0;
    x |= ((uint64_t)it_byte[0]) << W64_OFFSET_BYTE_0;
    x |= ((uint64_t)it_byte[1]) << W64_OFFSET_BYTE_1;
    x |= ((uint64_t)it_byte[2]) << W64_OFFSET_BYTE_2;
    x |= ((uint64_t)it_byte[3]) << W64_OFFSET_BYTE_3;
    x |= ((uint64_t)it_byte[4]) << W64_OFFSET_BYTE_4;
    x |= ((uint64_t)it_byte[5]) << W64_OFFSET_BYTE_5;
    x |= ((uint64_t)it_byte[6]) << W64_OFFSET_BYTE_6;
    x |= ((uint64_t)it_byte[7]) << W64_OFFSET_BYTE_7;

    return x;
}

/* ------------------------------------ - ----------------------------------- */
/* ------------------------------------ - ----------------------------------- */

// 16 BIT

inline uint16_t decode_little_endian_uint16(void* p_serialized_value)
{
    return get_16_bit_representation_(p_serialized_value);
}

inline int16_t decode_little_endian_int16(void* p_serialized_value)
{
    uint16_t x = get_16_bit_representation_(p_serialized_value);
    return *(int16_t*)(void*)&x;
}

// 32 BIT

inline uint32_t decode_little_endian_uint32(void* p_serialized_value)
{
    return get_32_bit_representation_(p_serialized_value);
}

inline int32_t decode_little_endian_int32(void* p_serialized_value)
{
    uint32_t x = get_32_bit_representation_(p_serialized_value);
    return *(int32_t*)(void*)&x;
}

inline float decode_little_endian_float32(void* p_serialized_value)
{
    uint32_t x = get_32_bit_representation_(p_serialized_value);
    return *(float*)(void*)&x;
}

// 64 BIT

inline uint64_t decode_little_endian_uint64(void* p_serialized_value)
{
    return get_64_bit_representation_(p_serialized_value);
}

inline int64_t decode_little_endian_int64(void* p_serialized_value)
{
    uint64_t x = get_64_bit_representation_(p_serialized_value);
    return *(int64_t*)(void*)&x;
}

inline double decode_little_endian_float64(void* p_serialized_value)
{
    uint64_t x = get_64_bit_representation_(p_serialized_value);
    return *(double*)(void*)&x;
}

#ifdef __cplusplus
 }
#endif


#endif