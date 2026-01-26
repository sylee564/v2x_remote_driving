#pragma once
#include <cstdint>
#include <cstddef>

namespace v2x { namespace crc16 {

enum class Variant : uint8_t {
  XMODEM,       // poly=0x1021, init=0x0000, xorout=0x0000
  CCITT_FALSE   // poly=0x1021, init=0xFFFF, xorout=0x0000
};

uint16_t compute(const uint8_t* data, size_t len, Variant v = Variant::XMODEM) noexcept;

// 편의 오버로드
inline uint16_t compute(const void* data, size_t len, Variant v = Variant::XMODEM) noexcept {
  return compute(static_cast<const uint8_t*>(data), len, v);
}

}} // namespace v2x::crc16
