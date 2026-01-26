#pragma once
#include <cstdint>
#include <array>
#include <stdexcept>
#include <limits>
#include <cmath>

namespace canbits {

enum class Endian { IntelLittle, MotorolaBig };

// ====== 비트 set (인코딩) ======
inline void put_bit(std::array<std::uint8_t,8>& p, std::size_t byte_idx, std::uint8_t bit_lsb, bool one){
    if (byte_idx >= 8 || bit_lsb > 7) throw std::out_of_range("bit out of range");
    const std::uint8_t m = static_cast<std::uint8_t>(1u << bit_lsb);
    if (one) p[byte_idx] = static_cast<std::uint8_t>(p[byte_idx] | m);
    else     p[byte_idx] = static_cast<std::uint8_t>(p[byte_idx] & ~m);
}

inline void put_unsigned_raw(std::array<std::uint8_t,8>& p, std::uint64_t raw,
                             std::uint8_t start_bit, std::uint8_t length,
                             Endian e = Endian::IntelLittle)
{
    if (length == 0 || length > 64) throw std::out_of_range("length 1..64");
    if (length < 64) raw &= ((1ULL << length) - 1ULL);

    if (e == Endian::IntelLittle) {
        for (std::uint8_t i=0;i<length;++i){
            const bool one = ((raw >> i) & 1ULL) != 0;
            const std::size_t bit_index = static_cast<std::size_t>(start_bit) + i;
            const std::size_t byte_idx  = bit_index / 8;
            const std::uint8_t bit_lsb  = static_cast<std::uint8_t>(bit_index % 8);
            if (byte_idx >= 8) throw std::out_of_range("write overflow (LE)");
            put_bit(p, byte_idx, bit_lsb, one);
        }
    } else {
        std::size_t byte_idx = start_bit / 8;
        int bit_msb = static_cast<int>(start_bit % 8); // 0=MSB..7=LSB
        int bit_lsb = 7 - bit_msb;
        for (std::uint8_t i=0;i<length;++i){
            const bool one = ((raw >> i) & 1ULL) != 0;
            if (byte_idx >= 8) throw std::out_of_range("write overflow (BE)");
            put_bit(p, byte_idx, static_cast<std::uint8_t>(bit_lsb), one);
            if (bit_lsb == 0){ bit_lsb = 7; ++byte_idx; } else { --bit_lsb; }
        }
    }
}

// ====== 비트 get (디코딩) ======
inline bool get_bit_lsb(const std::array<std::uint8_t,8>& p, std::size_t byte_idx, std::uint8_t bit_lsb){
    if (byte_idx >= 8 || bit_lsb > 7) throw std::out_of_range("bit out of range");
    return (p[byte_idx] >> bit_lsb) & 0x1u;
}

inline std::uint64_t get_unsigned_raw(const std::array<std::uint8_t,8>& p,
                                      std::uint8_t start_bit, std::uint8_t length,
                                      Endian e = Endian::IntelLittle)
{
    if (length == 0 || length > 64) throw std::out_of_range("length 1..64");
    std::uint64_t out = 0;

    if (e == Endian::IntelLittle) {
        for (std::uint8_t i=0;i<length;++i){
            const std::size_t bit_index = static_cast<std::size_t>(start_bit) + i;
            const std::size_t byte_idx  = bit_index / 8;
            const std::uint8_t bit_lsb  = static_cast<std::uint8_t>(bit_index % 8);
            if (byte_idx >= 8) throw std::out_of_range("read overflow (LE)");
            if (get_bit_lsb(p, byte_idx, bit_lsb)) out |= (1ULL << i);
        }
    } else {
        std::size_t byte_idx = start_bit / 8;
        int bit_msb = static_cast<int>(start_bit % 8);
        int bit_lsb = 7 - bit_msb;
        for (std::uint8_t i=0;i<length;++i){
            if (byte_idx >= 8) throw std::out_of_range("read overflow (BE)");
            if (get_bit_lsb(p, byte_idx, static_cast<std::uint8_t>(bit_lsb))) out |= (1ULL << i);
            if (bit_lsb == 0){ bit_lsb = 7; ++byte_idx; } else { --bit_lsb; }
        }
    }
    return out;
}

inline long long get_signed_raw(const std::array<std::uint8_t,8>& p,
                                std::uint8_t start_bit, std::uint8_t length,
                                Endian e = Endian::IntelLittle)
{
    const std::uint64_t u = get_unsigned_raw(p, start_bit, length, e);
    if (length == 64) return static_cast<long long>(u);
    const std::uint64_t sign = 1ULL << (length-1);
    if ((u & sign) == 0) return static_cast<long long>(u);
    const std::uint64_t mask = (sign<<1) - 1;
    return static_cast<long long>(u | (~mask));
}

// ====== 물리 변환 ======
inline double raw_to_phys_u(std::uint64_t raw, double factor, double offset){
    return static_cast<double>(raw)*factor + offset;
}
inline double raw_to_phys_s(long long raw, double factor, double offset){
    return static_cast<double>(raw)*factor + offset;
}

inline std::uint64_t phys_to_raw_u(double phys, double factor, double offset, std::uint8_t length){
    long double r = std::llround((phys - offset)/factor);
    if (r < 0) r = 0;
    long double maxu = (length==64)? std::numeric_limits<unsigned long long>::max() : (1ULL<<length)-1ULL;
    if (r > maxu) r = maxu;
    return static_cast<std::uint64_t>(r);
}
inline std::uint64_t phys_to_raw_s(double phys, double factor, double offset, std::uint8_t length){
    const long double r_ld = std::llround((phys - offset)/factor);
    const long long smin = -(1LL<<(length-1));
    const long long smax =  (1LL<<(length-1)) - 1LL;
    long long r = static_cast<long long>(std::max<long long>(smin, std::min<long long>(smax, static_cast<long long>(r_ld))));
    return static_cast<std::uint64_t>(r) & ((length==64)? ~0ULL : ((1ULL<<length)-1ULL));
}

} // namespace canbits
