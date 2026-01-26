// reports_decode.hpp
#pragma once
#include <array>
#include <cstdint>
#include "can_signal_io.hpp"  // get_unsigned_raw, get_signed_raw, raw_to_phys_*


namespace can_dec{
// ======================= 0x710 Steering =======================
struct SteeringReport710 {
    static constexpr std::uint32_t ID = 0x710;

    std::uint8_t eps_control_board{};
    std::uint8_t eps_en{};
    std::uint8_t eps_control_status{};
    std::uint8_t override_status{};
    double       steering_angle_deg{}; // factor 0.1, signed16 LE @ bytes2..3
    double       steer_drv_tq_Nm{};    // (raw-2048)*0.01, 12-bit: b4(8) + b5[0..3]
    double       steer_out_tq_Nm{};    // (raw-2048)*0.01, 12-bit: b5[4..7] + b6(8)
    std::uint8_t eps_alive_cnt{};

    void parse(const std::array<std::uint8_t,8>& p) {
        using namespace canbits;

        // byte0
        eps_en             = static_cast<std::uint8_t>(get_unsigned_raw(p, 0, 1, Endian::IntelLittle));
        eps_control_board  = static_cast<std::uint8_t>(get_unsigned_raw(p, 1, 3, Endian::IntelLittle));
        // byte1
        eps_control_status = static_cast<std::uint8_t>(get_unsigned_raw(p, 8, 4, Endian::IntelLittle));
        override_status    = static_cast<std::uint8_t>(get_unsigned_raw(p, 13, 1, Endian::IntelLittle));

        // steering angle: LE16 @ bytes2..3, scale 0.1
        {
            const long long raw = get_signed_raw(p, 16, 16, Endian::IntelLittle);
            steering_angle_deg = raw_to_phys_s(raw, 0.1, 0.0);
        }

        // steer_drv_tq: 12-bit (byte4 LSB8 + byte5[0..3] MS4), all IntelLittle
        {
            const std::uint64_t low8 = get_unsigned_raw(p, 32, 8, Endian::IntelLittle);  // byte4
            const std::uint64_t hi4  = get_unsigned_raw(p, 40, 4, Endian::IntelLittle);  // byte5[0..3]
            const std::uint64_t raw12 = (hi4 << 8) | low8;
            steer_drv_tq_Nm = static_cast<double>(static_cast<long long>(raw12) - 2048) * 0.01;
        }

        // steer_out_tq: 12-bit (byte5[4..7] LSB4 + byte6 MS8), IntelLittle
        {
            const std::uint64_t low4 = get_unsigned_raw(p, 44, 4, Endian::IntelLittle);  // byte5 upper nibble
            const std::uint64_t hi8  = get_unsigned_raw(p, 48, 8, Endian::IntelLittle);  // byte6
            const std::uint64_t raw12 = (hi8 << 4) | low4;
            steer_out_tq_Nm = static_cast<double>(static_cast<long long>(raw12) - 2048) * 0.01;
        }

        // alive count: byte7
        eps_alive_cnt = static_cast<std::uint8_t>(get_unsigned_raw(p, 56, 8, Endian::IntelLittle));
    }
};

// ======================= 0x711 ACC =======================
struct AccReport711 {
    static constexpr std::uint32_t ID = 0x711;

    std::uint8_t acc_en_status{};
    std::uint8_t acc_control_board{};
    std::uint8_t acc_user_can_err{};
    std::uint8_t acc_veh_err{};
    std::uint8_t acc_err{};
    std::uint8_t acc_control_status{};
    std::uint8_t vehicle_speed{};
    double       long_accel_mps2{}; // (raw-1023)*0.01, 11-bit
    std::uint8_t acc_alive_cnt{};

    void parse(const std::array<std::uint8_t,8>& p) {
        using namespace canbits;

        // byte0
        acc_en_status      = static_cast<std::uint8_t>(get_unsigned_raw(p, 0, 1, Endian::IntelLittle));
        acc_control_board  = static_cast<std::uint8_t>(get_unsigned_raw(p, 1, 3, Endian::IntelLittle));
        acc_user_can_err   = static_cast<std::uint8_t>(get_unsigned_raw(p, 4, 1, Endian::IntelLittle));
        acc_veh_err        = static_cast<std::uint8_t>(get_unsigned_raw(p, 6, 1, Endian::IntelLittle));
        acc_err            = static_cast<std::uint8_t>(get_unsigned_raw(p, 7, 1, Endian::IntelLittle));

        // byte1
        acc_control_status = static_cast<std::uint8_t>(get_unsigned_raw(p, 8, 4, Endian::IntelLittle));

        // vehicle_speed: byte2
        vehicle_speed      = static_cast<std::uint8_t>(get_unsigned_raw(p, 16, 8, Endian::IntelLittle));

        // long_accel: 11-bit LE: byte4(8) + byte5
        {
            const std::uint64_t low8 = get_unsigned_raw(p, 32, 8, Endian::IntelLittle); // b4
            const std::uint64_t hi3  = get_unsigned_raw(p, 40, 3, Endian::IntelLittle); // b5[0..2]
            const std::uint64_t raw11 = (hi3 << 8) | low8;
            long_accel_mps2 = static_cast<double>(static_cast<long long>(raw11) - 1023) * 0.01;
        }

        acc_alive_cnt = static_cast<std::uint8_t>(get_unsigned_raw(p, 56, 8, Endian::IntelLittle));
    }
};

// ======================= 0x712 Wheel Speed =======================
struct WheelReport712 {
    static constexpr std::uint32_t ID = 0x712;
    static constexpr double kFactor = 0.03125;

    double wheel_spd_fl{};
    double wheel_spd_fr{};
    double wheel_spd_rl{};
    double wheel_spd_rr{};

    void parse(const std::array<std::uint8_t,8>& p) {
        using namespace canbits;
        // 16-bit LE words: (0..1), (2..3), (4..5), (6..7)
        wheel_spd_fl = raw_to_phys_u(get_unsigned_raw(p,  0, 16, Endian::IntelLittle), kFactor, 0.0);
        wheel_spd_fr = raw_to_phys_u(get_unsigned_raw(p, 16, 16, Endian::IntelLittle), kFactor, 0.0);
        wheel_spd_rl = raw_to_phys_u(get_unsigned_raw(p, 32, 16, Endian::IntelLittle), kFactor, 0.0);
        wheel_spd_rr = raw_to_phys_u(get_unsigned_raw(p, 48, 16, Endian::IntelLittle), kFactor, 0.0);
    }
};

// ======================= 0x713 Brake =======================
struct BrakeReport713 {
    static constexpr std::uint32_t ID = 0x713;

    double lat_accel_mps2{};     // (raw-1023)*0.01  from bytes0..1 (LE16)
    double yaw_rate_dps{};       // (raw-4095)*0.01  from bytes3..4 (LE16)
    double brake_cylinder_kpa{}; // raw * 0.1        from bytes6..7 (LE16)

    void parse(const std::array<std::uint8_t,8>& p) {
        using namespace canbits;

        lat_accel_mps2     = static_cast<double>(static_cast<long long>(
                                 get_unsigned_raw(p,  0, 16, Endian::IntelLittle)) - 1023) * 0.01;
        yaw_rate_dps       = static_cast<double>(static_cast<long long>(
                                 get_unsigned_raw(p, 24, 16, Endian::IntelLittle)) - 4095) * 0.01;
        brake_cylinder_kpa = raw_to_phys_u(get_unsigned_raw(p, 48, 16, Endian::IntelLittle), 0.1, 0.0);
    }
};

// ======================= 0x720 Gear =======================
struct GearReport720 {
    static constexpr std::uint32_t ID = 0x720;

    std::uint8_t gear_cur{};            // byte0 상위 4비트
    std::uint8_t gear_cmd{};            // byte1 상위 4비트
    std::uint8_t gear_control_status{}; // byte2 상위 4비트

    void parse(const std::array<std::uint8_t,8>& p) {
        using namespace canbits;
        // 상위 4비트(MSB nibble)를 IntelLittle 전역 인덱스로 읽어도 동일하게 동작
        gear_cur            = static_cast<std::uint8_t>(get_unsigned_raw(p,  0, 4, Endian::IntelLittle)); // byte0 [7..4]
        gear_cmd            = static_cast<std::uint8_t>(get_unsigned_raw(p,  8, 4, Endian::IntelLittle)); // byte1 [7..4]
        gear_control_status = static_cast<std::uint8_t>(get_unsigned_raw(p, 16, 4, Endian::IntelLittle)); // byte2 [7..4]
    }
};
}