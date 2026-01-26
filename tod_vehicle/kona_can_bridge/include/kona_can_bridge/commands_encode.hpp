// commands_encode.hpp
#pragma once
#include <array>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include "can_signal_io.hpp"   // put_unsigned_raw, phys_to_raw_s, Endian::IntelLittle

namespace can_enc {

// 공통: 앞 7바이트 합의 LSB (필요 시 규격에 맞게 수정)
inline std::uint8_t checksum_sum8_first7(const std::array<std::uint8_t,8>& p) {
    std::uint32_t s = 0;
    for (int i = 0; i < 7; ++i) s += p[i];
    return static_cast<std::uint8_t>(s & 0xFF);
}

// ========================= PrimaryCommand (0x157) =========================
struct PrimaryCommand157Encoder {
    static constexpr std::uint32_t ID = 0x157;
    std::array<std::uint8_t,8> payload{};

    void reset() { payload.fill(0); }
    std::uint8_t*       data()       { return payload.data(); }
    const std::uint8_t* data() const { return payload.data(); }

    // 기존 API 유지 (외부에서 checksum 제공)
    void UpdateData(int steer_angle_target_deg, float acc_dec_cmd, std::uint8_t checksum_157) {
        set_steer_angle_target(steer_angle_target_deg);
        set_accel_dec_cmd(acc_dec_cmd);
        set_checksum_157(checksum_157);
    }


    // steer_angle_target: clamp [-360, 360], factor 0.1, signed16, LE @ bytes0..1
    void set_steer_angle_target(int steer_angle_target_deg) {
        using namespace canbits;
        const int v = std::clamp(steer_angle_target_deg, -360, 360);
        const std::uint64_t raw = phys_to_raw_s(static_cast<double>(v), 0.1, 0.0, 16);
        put_unsigned_raw(payload, raw, /*start_bit*/0, /*len*/16, Endian::IntelLittle);
    }

    // acc_dec_cmd: clamp [-3.00, +1.00], floor(centi) + 10.23, *100 → LE16 @ bytes3..4
    void set_accel_dec_cmd(float acc_dec_cmd) {
        using namespace canbits;
        double a = std::clamp(static_cast<double>(acc_dec_cmd), -3.00, 1.00);
        const double q = std::floor(a * 100.0) / 100.0;  // 소수 둘째 자리 내림
        const int raw_i = static_cast<int>(std::lround((q + 10.23) * 100.0));
        const std::uint16_t raw = static_cast<std::uint16_t>(std::clamp(raw_i, 0, 65535));
        put_unsigned_raw(payload, raw, /*start_bit*/24, /*len*/16, Endian::IntelLittle);
    }

    // data[7]
    void set_checksum_157(std::uint8_t checksum_157) {
        using namespace canbits;
        put_unsigned_raw(payload, checksum_157, /*start_bit*/56, /*len*/8, Endian::IntelLittle);
    }
};

// ========================= SecondCommand (0x156) =========================
struct Secondcommand156Encoder {
    static constexpr std::uint32_t ID = 0x156;
    std::array<std::uint8_t,8> payload{};

    void reset() { payload.fill(0); }
    std::uint8_t*       data()       { return payload.data(); }
    const std::uint8_t* data() const { return payload.data(); }

    // 기존 API 유지
    void UpdateData(std::uint8_t eps_en, std::uint8_t override_ignore, std::uint8_t eps_speed,
                    std::uint8_t acc_en, std::uint8_t aeb_en, std::uint8_t indicator,
                    std::uint8_t gear_cmd, std::uint8_t checksum_156)
    {
        set_override_ignore(override_ignore);
        set_eps_en(eps_en);
        set_eps_speed(eps_speed);
        set_aeb_en(aeb_en);         // byte2 bit6로 반영
        set_acc_en(acc_en);
        set_gear_cmd(gear_cmd);
        set_indicator_cmd(indicator);
        set_checksum_156(checksum_156);
    }


    // data[0] bit2
    void set_override_ignore(std::uint8_t override_ignore) {
        using namespace canbits;
        // 먼저 해당 비트를 0으로 쓴 뒤, 필요하면 1로 (put_unsigned_raw는 알아서 0/1 모두 씀)
        put_unsigned_raw(payload, (override_ignore ? 1ULL : 0ULL), /*start_bit*/2, /*len*/1, Endian::IntelLittle);
    }

    // data[0] bit0
    void set_eps_en(std::uint8_t eps_en) {
        using namespace canbits;
        put_unsigned_raw(payload, (eps_en ? 1ULL : 0ULL), /*start_bit*/0, /*len*/1, Endian::IntelLittle);
    }

    // data[1] = eps_speed
    void set_eps_speed(std::uint8_t eps_speed) {
        using namespace canbits;
        put_unsigned_raw(payload, eps_speed, /*start_bit*/8, /*len*/8, Endian::IntelLittle);
    }

    // data[2] bit6 (원래 주석되어 있던 AEB EN 위치를 명확히 bit6로 가정)
    // 실제 DBC가 다르면 start_bit만 바꿔주세요.
    void set_aeb_en(std::uint8_t aeb_en) {
        using namespace canbits;
        put_unsigned_raw(payload, (aeb_en ? 1ULL : 0ULL), /*start_bit*/22, /*len*/1, Endian::IntelLittle);
    }

    // data[2] bit0
    void set_acc_en(std::uint8_t acc_en) {
        using namespace canbits;
        put_unsigned_raw(payload, (acc_en ? 1ULL : 0ULL), /*start_bit*/16, /*len*/1, Endian::IntelLittle);
    }

    // data[5] 상위 4비트(7..4) — 값 4비트 전체를 정확히 덮어씀
    void set_gear_cmd(std::uint8_t gear_cmd) {
        using namespace canbits;
        const std::uint64_t raw4 = (gear_cmd & 0x0Fu);
        put_unsigned_raw(payload, raw4, /*start_bit*/44, /*len*/4, Endian::IntelLittle);
    }

    // data[5] 하위 니블 특수 매핑: 1→bit1, 2→bit2, 3→bit0 (다시 호출해도 안전하도록 먼저 하위 3비트 클리어)
    void set_indicator_cmd(std::uint8_t indicator) {
        using namespace canbits;
        // 하위 3비트(40,41,42)를 먼저 0으로 만든다.
        // put_unsigned_raw는 길이가 1인 비트를 0/1로 각각 써주므로, 빈번 호출에도 안전.
        put_unsigned_raw(payload, 0ULL, 40, 1, Endian::IntelLittle);
        put_unsigned_raw(payload, 0ULL, 41, 1, Endian::IntelLittle);
        put_unsigned_raw(payload, 0ULL, 42, 1, Endian::IntelLittle);

        if      (indicator == 1) put_unsigned_raw(payload, 1ULL, 41, 1, Endian::IntelLittle); // bit1
        else if (indicator == 2) put_unsigned_raw(payload, 1ULL, 42, 1, Endian::IntelLittle); // bit2
        else if (indicator == 3) put_unsigned_raw(payload, 1ULL, 40, 1, Endian::IntelLittle); // bit0
        // 그 외(0 포함)는 모두 0 유지
    }

    // data[7]
    void set_checksum_156(std::uint8_t checksum_156) {
        using namespace canbits;
        put_unsigned_raw(payload, checksum_156, /*start_bit*/56, /*len*/8, Endian::IntelLittle);
    }
};

} // namespace can_enc
