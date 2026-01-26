#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

#if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
#endif

// ---- pack(1) 보호 ----
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
  #pragma pack(push,1)
#endif

#include "vendor/v2x_app_ext.h"
#include "vendor/nr_v2x_interface.h"
#include "vendor/db_v2x.h"

#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
  #pragma pack(pop)
#endif

#include "nr_v2x_helper/crc16.hpp"

#if defined(__GNUC__) || defined(__clang__)
  #pragma GCC diagnostic pop
#endif

// ====================== 레이아웃 검증 ======================

// App header: data 오프셋 10 고정
static_assert(offsetof(V2x_App_Hdr, data) == 10, "V2x_App_Hdr.data offset must be 10");
static_assert(sizeof(((V2x_App_Hdr*)0)->magic) == 4, "magic must be 4 bytes");

// Tx/Rx fixed
static_assert(sizeof(V2x_App_TxMsg) == 21, "V2x_App_TxMsg fixed must be 21");
static_assert(sizeof(V2x_App_RxMsg) == 22, "V2x_App_RxMsg fixed must be 22");

// TLVC 헤더는 on-wire 6바이트(= data 오프셋이 6이면 OK)
static_assert(offsetof(V2x_App_Ext_TLVC, data) == 6,
              "V2x_App_Ext_TLVC.data must start at offset 6 (type+len=6)");
// sizeof(...)은 컴파일러 패딩으로 8일 수 있으니 검사하지 않음

// Overall sizes
static_assert(sizeof(TLVC_Overall)    == 16, "TLVC_Overall must be 16");
static_assert(sizeof(TLVC_Overall_V2) == 17, "TLVC_Overall_V2 must be 17");

// ---- 벤더 SIZE_* 매크로 정합성/강제 ----
#if defined(SIZE_V2X_APP_EXT_HEADER)
  static_assert(SIZE_V2X_APP_EXT_HEADER == 10, "SIZE_V2X_APP_EXT_HEADER must be 10");
#endif
#if defined(SIZE_TX_HEADER)
  static_assert(SIZE_TX_HEADER == 21, "SIZE_TX_HEADER must be 21");
#endif
#if defined(SIZE_RX_HEADER)
  static_assert(SIZE_RX_HEADER == 22, "SIZE_RX_HEADER must be 22");
#endif

// 문제의 매크로: 벤더가 8로 정의했을 수 있음 → 6으로 강제 재정의
#ifdef SIZE_EXT_HEADER
  #undef SIZE_EXT_HEADER
#endif
#define SIZE_EXT_HEADER 6
// 더 이상 SIZE_EXT_HEADER에 대한 static_assert는 두지 않음(이미 강제했으므로)

// TX/RX packet overhead (있으면 확인)
#if defined(SIZE_TX_PACKET_OVERHEAD)
  static_assert(SIZE_TX_PACKET_OVERHEAD == (10 + 21 + 2), "SIZE_TX_PACKET_OVERHEAD must be 33");
#endif
#if defined(SIZE_RX_PACKET_OVERHEAD)
  static_assert(SIZE_RX_PACKET_OVERHEAD == (10 + 22 + 2), "SIZE_RX_PACKET_OVERHEAD must be 34");
#endif

// DB_V2X fixed 40 바이트 (필드명이 다르면 주석)
namespace v2x_vendor_detail {
  constexpr std::size_t DB_V2X_FIXED_LEN = offsetof(DB_V2X_T, payload);
}
static_assert(v2x_vendor_detail::DB_V2X_FIXED_LEN == 40, "DB_V2X fixed header must be 40");
static_assert(offsetof(DB_V2X_T, ulPayloadCrc32) + sizeof(((DB_V2X_T*)0)->ulPayloadCrc32) == 40,
              "DB_V2X ulPayloadCrc32 end offset must be 40");

// ====================== 재노출 유틸/상수 ======================
namespace v2x_vendor {
  constexpr std::size_t kHdrBytes        = 10;
  constexpr std::size_t kTxFixedBytes    = 21;
  constexpr std::size_t kRxFixedBytes    = 22;
  constexpr std::size_t kTlvcHeaderBytes = 6;   // on-wire
  constexpr std::size_t kOverallV1Bytes  = 16;
  constexpr std::size_t kOverallV2Bytes  = 17;
  constexpr std::size_t kTailCrcBytes    = 2;
  constexpr std::size_t kDbV2xFixedBytes = 40;

  inline uint16_t crc16_tlvc(const void* tlv_without_crc, std::size_t tlv_len_without_crc) {
    return v2x::crc16::compute(tlv_without_crc, tlv_len_without_crc,
                               v2x::crc16::Variant::XMODEM);
  }
  inline uint16_t crc16_packet_from_len_field(const V2x_App_Hdr* p, std::size_t total_packet_bytes) {
    const auto* start = reinterpret_cast<const uint8_t*>(&p->len); // offset 4
    const std::size_t n = (total_packet_bytes >= 6) ? (total_packet_bytes - 6) : 0;
    return v2x::crc16::compute(start, n, v2x::crc16::Variant::XMODEM);
  }
  constexpr std::size_t total_tx_packet_bytes(std::size_t ext_payload_bytes) {
    return kHdrBytes + kTxFixedBytes + ext_payload_bytes + kTailCrcBytes;
  }
  constexpr std::size_t wire_len_field_value(std::size_t total_packet_bytes) {
    return (total_packet_bytes >= 6) ? (total_packet_bytes - 6) : 0;
  }
}
