#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <cstring>
#include <algorithm>
#include <array>
#include <functional>
#include <sstream>
#include <unordered_map>
#include <limits>
#include <chrono>
#include <iostream>

#include <nr_v2x_helper/byte_order.hpp>
#include <nr_v2x_helper/crc16.hpp>
#include "nr_v2x_interface/protocol/vendor_protocol.hpp"

#ifndef V2X_RXPARSER_DEBUG
  #define V2X_RXPARSER_WARN(x) do{/*no-op*/}while(0)
#else
  #define V2X_RXPARSER_WARN(x) do{ std::cerr << "[rx_parser] " << x << std::endl; }while(0)
#endif

namespace v2x { namespace proto { namespace wire {

// ---------- Packet header ----------
struct PacketHeader { uint16_t total_len{}, seq{}, payload_id{}; };
enum class TailCrcPolicy : uint8_t { RequireValid, AllowZero, Skip };

inline bool peek_total_len(const uint8_t* buf, std::size_t have, std::size_t& out_total)
{
  out_total = 0;
  if (!buf || have < 4) return false;

  if (std::memcmp(buf, V2X_INF_EXT_MAGIC, 4) != 0) {
    std::size_t i = 1;
    for (; i + 3 < have; ++i) {
      if (std::memcmp(buf + i, V2X_INF_EXT_MAGIC, 4) == 0) break;
    }
    out_total = i;                 // resync skip bytes
    return true;
  }

  if (have < 6) return false;

  const uint16_t wire_len = v2x::get_be16(buf + 4);
  const std::size_t total = static_cast<std::size_t>(wire_len) + 6u;

  if (total < 12u) { out_total = 1; return true; }
  out_total = total;
  return true;
}

inline bool peek_total_len_no_tail(const uint8_t* buf, std::size_t n, std::size_t& total_out) {
  if (!buf || n < 6) return false;
  if (std::memcmp(buf, V2X_INF_EXT_MAGIC, 4) != 0) { total_out = 1; return true; }
  const uint16_t len = v2x::get_be16(buf + 4);
  total_out = 4u + 2u + static_cast<std::size_t>(len); // magic+len+payload (tail CRC 제외)
  return true;
}

inline bool parse_and_verify_header(const uint8_t* buf, std::size_t total,
                                    PacketHeader& out,
                                    TailCrcPolicy tail = TailCrcPolicy::RequireValid)
{
  if (!buf || total < 12) return false;
  if (std::memcmp(buf, V2X_INF_EXT_MAGIC, 4) != 0) return false;

  const uint16_t len_be = v2x::get_be16(buf + 4);
  if (static_cast<std::size_t>(len_be) + 6u != total) return false;

  if (tail != TailCrcPolicy::Skip) {
    const uint16_t crc_wire = v2x::get_be16(buf + total - 2);
    if (!(tail == TailCrcPolicy::AllowZero && crc_wire == 0x0000)) {
      const uint16_t crc_calc = v2x::crc16::compute(buf + 4, total - 6, v2x::crc16::Variant::XMODEM);
      if (crc_calc != crc_wire) return false;
    }
  }
  out.total_len  = static_cast<uint16_t>(total);
  out.seq        = v2x::get_be16(buf + 6);
  out.payload_id = v2x::get_be16(buf + 8);
  return true;
}

// ---------- Rx container ----------
struct RxMsgView {
  uint32_t psid{};   // be32
  uint8_t  castMode{};
  uint32_t srcId{}, dstId{};
  uint8_t  rcpi{};
  const uint8_t* data=nullptr; std::size_t data_len=0; bool ok=false;
};

inline RxMsgView parse_rx_container(const uint8_t* p, std::size_t n) {
  constexpr std::size_t overhead = 22; // psid4 + cast1 + src4 + dst4 + reserved8 + rcpi1
  RxMsgView v{};
  if (!p || n < overhead) return v;
  v.psid     = v2x::get_be32(p+0);
  v.castMode = *(p+4);
  v.srcId    = v2x::get_be32(p+5);
  v.dstId    = v2x::get_be32(p+9);
  v.rcpi     = *(p+21);
  v.data     = p + overhead;
  v.data_len = n - overhead;
  v.ok = true;
  return v;
}

// ---------- TLVC ----------
struct TlvcView {
  uint32_t type{}; uint16_t len{};
  const uint8_t* base{};       // TLVC 시작(type)
  const uint8_t* v{}; 
  std::size_t v_len{};
  const uint8_t* crc_ptr{}; 
  std::size_t total_len{};     // 6 + (L - 2)  == T+L+V  (CRC 2B 제외)
  bool ok=false;
};

inline bool tlvc_next(const uint8_t*& cur, std::size_t& remain, TlvcView& out) {
  out = TlvcView{};
  if (!cur || remain < 8) return false;

  const uint16_t L = v2x::get_be16(cur + 4);
  const std::size_t total_no_crc = 6u + static_cast<std::size_t>(L) - 2u; // (T+L) + V
  if (L < 2u || 6u + static_cast<std::size_t>(L) > remain) return false;

  out.type      = v2x::get_be32(cur + 0);
  out.len       = L;
  out.base      = cur;
  out.v         = cur + 6u;
  out.v_len     = static_cast<std::size_t>(L) - 2u; // V only
  out.crc_ptr   = cur + 6u + out.v_len;             // CRC 위치
  out.total_len = total_no_crc;                     // CRC 제외 전체(T+L+V)
  out.ok        = true;

  cur    += (6u + static_cast<std::size_t>(L));     // T+L+V+CRC 모두 소비
  remain -= (6u + static_cast<std::size_t>(L));
  return true;
}

inline bool tlvc_check_crc_relaxed(const TlvcView& tv,
                                   v2x::crc16::Variant primary = v2x::crc16::Variant::XMODEM,
                                   bool also_try_v_only = true,
                                   bool also_try_ccitt_false = false)
{
  if (!tv.ok || tv.len < 2) return false;

  const uint16_t wire = v2x::get_be16(tv.crc_ptr);
  if (wire == 0x0000) return true;

  const uint8_t* tlv_ptr = tv.base;               // T(4)+L(2)+V
  const size_t   full_n  = 6u + tv.v_len;         // CRC 제외
  const uint8_t* v_ptr   = tv.v;
  const size_t   v_n     = tv.v_len;

  const uint16_t calc_full = v2x::crc16::compute(tlv_ptr, full_n, primary);
  if (calc_full == wire) return true;

  if (also_try_v_only) {
    const uint16_t calc_v = v2x::crc16::compute(v_ptr, v_n, primary);
    if (calc_v == wire) return true;
  }

  if (also_try_ccitt_false) {
    const uint16_t c2 = v2x::crc16::compute(tlv_ptr, full_n, v2x::crc16::Variant::CCITT_FALSE);
    if (c2 == wire) return true;
    if (also_try_v_only) {
      const uint16_t c2v = v2x::crc16::compute(v_ptr, v_n, v2x::crc16::Variant::CCITT_FALSE);
      if (c2v == wire) return true;
    }
  }
  return false;
}

}}} // ns wire

// ---------- DB_V2X fixed header ----------
namespace v2x { namespace proto { namespace common {
struct DbV2xFixed {
  uint16_t eDeviceType{}, eTeleCommType{}, eServiceId{}, eActionType{}, eRegionId{}, ePayloadType{}, eCommId{};
  uint32_t unDeviceId{}, ulPayloadLength{};
  uint64_t ulTimeStamp{};
  uint16_t usDbVer{}, usHwVer{}, usSwVer{};
  bool ok=false;
};
inline DbV2xFixed parse_dbv2x_fixed(const uint8_t* p, std::size_t n) {
  DbV2xFixed d{};
  if (!p || n < 40) return d;
  d.eDeviceType     = v2x::get_be16(p);
  d.eTeleCommType   = v2x::get_be16(p+2);
  d.unDeviceId      = v2x::get_be32(p+4);
  d.ulTimeStamp     = v2x::get_be64(p+8);
  d.eServiceId      = v2x::get_be16(p+16);
  d.eActionType     = v2x::get_be16(p+18);
  d.eRegionId       = v2x::get_be16(p+20);
  d.ePayloadType    = v2x::get_be16(p+22);
  d.eCommId         = v2x::get_be16(p+24);
  d.usDbVer         = v2x::get_be16(p+26);
  d.usHwVer         = v2x::get_be16(p+28);
  d.usSwVer         = v2x::get_be16(p+30);
  d.ulPayloadLength = v2x::get_be32(p+32);
  d.ok = true;
  return d;
}
}}}

// ---------- RAW-MUX (VIDEO) reassembler ----------
namespace v2x { namespace proto { namespace rawmux {

#pragma pack(push, 1)
struct AppMuxHdr {
  uint8_t  kind;       // 2 = VIDEO
  uint8_t  stream;     // 0,1,2...
  uint8_t  flags;      // bit0: keyframe
  uint8_t  _pad;
  uint32_t frame_seq;  // BE
  uint16_t frag_idx;   // BE
  uint16_t frag_cnt;   // BE
  uint32_t frame_len;  // BE
};
#pragma pack(pop)

inline uint32_t be32(const void* p){ return v2x::get_be32(reinterpret_cast<const uint8_t*>(p)); }
inline uint16_t be16(const void* p){ return v2x::get_be16(reinterpret_cast<const uint8_t*>(p)); }

struct Inflight {
  bool key=false; 
  uint8_t stream=0;
  uint32_t fseq=0; 
  uint32_t frame_len=0;
  uint16_t frag_cnt=0;
  uint32_t frag_bytes=0;          // learned fragment payload bytes for this frame
  std::vector<uint8_t> buf;
  std::vector<uint8_t> have;
  uint32_t filled=0;
  std::chrono::steady_clock::time_point started;

  Inflight() = default;
  Inflight(uint8_t s, bool k, uint32_t fs, uint32_t fl, uint16_t fc)
    : key(k), stream(s), fseq(fs), frame_len(fl), frag_cnt(fc),
      buf(fl), have(fc, 0), filled(0), started(std::chrono::steady_clock::now()) {}
};

struct RxFragBank {
  std::unordered_map<uint8_t, std::unordered_map<uint32_t, Inflight>> in;
  std::chrono::milliseconds assemble_timeout{100};

  template<typename EmitCb>
  void feed_bytes(const uint8_t* v, size_t n, EmitCb&& emit) {
    size_t off = 0;
    while (off + sizeof(AppMuxHdr) <= n) {
      const auto* h = reinterpret_cast<const AppMuxHdr*>(v + off);
      if (h->kind != 2) { ++off; continue; }

      const uint8_t  stream = h->stream;
      const bool     key    = (h->flags & 0x01) != 0;
      const uint32_t fseq   = be32(&h->frame_seq);
      const uint16_t findex = be16(&h->frag_idx);
      const uint16_t fcnt   = std::max<uint16_t>(1, be16(&h->frag_cnt));
      const uint32_t flen   = be32(&h->frame_len);
      if (flen == 0 || fcnt == 0 || findex >= fcnt) break;

      const uint8_t* p = v + off + sizeof(AppMuxHdr);
      const size_t   remain = n - off - sizeof(AppMuxHdr);
      if (remain == 0) break;

      auto& tab = in[stream];
      auto it = tab.find(fseq);
      if (it == tab.end()) it = tab.emplace(fseq, Inflight(stream, key, fseq, flen, fcnt)).first;
      auto& f = it->second;

      if (f.frame_len != flen || f.frag_cnt != fcnt) {
        tab.erase(it);
        it = tab.emplace(fseq, Inflight(stream, key, fseq, flen, fcnt)).first;
      }
      auto& cur = it->second;

      if (cur.frag_bytes == 0 || findex == 0) {
        cur.frag_bytes = static_cast<uint32_t>(std::max<size_t>(200, remain));
      }

      const size_t off2 = static_cast<size_t>(findex) * static_cast<size_t>(cur.frag_bytes);
      if (off2 >= cur.buf.size()) { tab.erase(fseq); break; }

      const size_t copy_n = std::min(remain, cur.buf.size() - off2);
      if (copy_n == 0) { tab.erase(fseq); break; }

      std::memcpy(cur.buf.data() + off2, p, copy_n);
      if (findex < cur.have.size() && cur.have[findex] == 0) {
        cur.have[findex] = 1;
        cur.filled += static_cast<uint32_t>(copy_n);
      }

      const bool all_frags = std::all_of(cur.have.begin(), cur.have.end(), [](uint8_t b){ return b!=0; });
      if (all_frags || cur.filled >= cur.frame_len) {
        const size_t valid_len = std::min<size_t>(cur.filled, cur.frame_len);
        emit(cur.stream, cur.key, cur.fseq, cur.buf.data(), valid_len);
        tab.erase(fseq);
      } else {
        // timeout drop
        if (std::chrono::steady_clock::now() - cur.started > assemble_timeout) {
          tab.erase(fseq);
        }
      }
      break; // assume 1 fragment per TLVC
    }
  }
};

}}} // ns v2x::proto::rawmux

// ---------- RX 디스패처 ----------
namespace v2x { namespace proto {

using VideoTlvcHandler   = std::function<void(const v2x::proto::wire::TlvcView&)>;
using RawPayloadHandler  = std::function<void(uint32_t tlvc_type, const uint8_t* p, size_t n)>;

// STATUS: vendor TLVC 포인터 직접 전달
using OnTxModemV2   = std::function<void(const TLVC_STATUS_Tx_ModemUnit_V2*)>;
using OnTxModemV1   = std::function<void(const TLVC_STATUS_Tx_ModemUnit*)>;
using OnRxModemV2   = std::function<void(const TLVC_STATUS_Rx_ModemUnit_V2*)>;
using OnRxModemV1   = std::function<void(const TLVC_STATUS_Rx_ModemUnit*)>;
using OnCommUnitV2  = std::function<void(const TLVC_STATUS_CommUnit_V2*)>;
using OnCommUnitV1  = std::function<void(const TLVC_STATUS_CommUnit*)>;
using OnCtrlUnitV2  = std::function<void(const TLVC_STATUS_ControlUnit_V2*)>;
using OnCtrlUnitV1  = std::function<void(const TLVC_STATUS_ControlUnit*)>;

// SSOV: 메타 + 페이로드 포인터 전달
using SsovHandler   = std::function<void(const common::DbV2xFixed&, const uint8_t* payload, size_t len)>;

struct RxHandlers {
  VideoTlvcHandler  on_video;
  RawPayloadHandler on_raw;

  OnTxModemV2  on_status_tx_modem_v2;
  OnTxModemV1  on_status_tx_modem_v1;
  OnRxModemV2  on_status_rx_modem_v2;
  OnRxModemV1  on_status_rx_modem_v1;
  OnCommUnitV2 on_status_comm_unit_v2;
  OnCommUnitV1 on_status_comm_unit_v1;
  OnCtrlUnitV2 on_status_ctrl_unit_v2;
  OnCtrlUnitV1 on_status_ctrl_unit_v1;

  SsovHandler  on_ssov;
};

struct RxOptions {
  bool status_only_when_ssov = true; // SSOV 있을 때만 STATUS 이벤트 허용
  bool strict_crc = false;           // TLVC CRC 엄격 검사
  bool vehicle_mode = false;         // (참고 플래그, 파서에서는 미사용)
};

inline bool handle_wsc_payload(const uint8_t* payload, std::size_t payload_len,
                               v2x::crc16::Variant crc_variant,
                               /*out*/ uint8_t& action_result,
                               /*out*/ uint32_t& psid)
{
  using namespace v2x;
  if (!payload || payload_len < sizeof(V2x_App_WSC_Add_Crc))
    return false;
  if (payload_len != sizeof(V2x_App_WSC_Add_Crc)) {
    V2X_RXPARSER_WARN("WSC payload length unexpected=" << payload_len);
    return false;
  }

  action_result = payload[0];
  psid          = v2x::get_be32(payload + 1);
  const uint16_t inner_wire = v2x::get_be16(payload + 5);

  const uint16_t inner_calc = v2x::crc16::compute(payload, 5u, crc_variant);
  if (inner_wire != inner_calc) {
    V2X_RXPARSER_WARN("WSC inner CRC mismatch");
    return false;
  }
  return true;
}

// SSOV TLVC -> meta + payload 포인터
inline bool parse_ssov_tlvc(const v2x::proto::wire::TlvcView& tv,
                            common::DbV2xFixed& meta,
                            const uint8_t*& payload_ptr, size_t& payload_len)
{
  payload_ptr = nullptr; payload_len = 0;
  if (tv.type != EM_PT_SSOV || tv.v_len < 40) return false;
  meta = common::parse_dbv2x_fixed(tv.v, tv.v_len);
  if (!meta.ok) return false;

  const uint8_t* p = tv.v + 40;
  size_t n = tv.v_len - 40;
  if (meta.ulPayloadLength > 0 && meta.ulPayloadLength <= n) n = static_cast<size_t>(meta.ulPayloadLength);
  if (n == 0) return false;
  payload_ptr = p; payload_len = n;
  return true;
}

// 단순화된 RX 디스패치 함수 (publish 없음, 콜백만 호출)
inline bool dispatch_rxmsg(const uint8_t* payload, std::size_t paylen,
                           const RxHandlers& handlers, const RxOptions& opts)
{
  using namespace v2x::proto::wire;
  if (!payload || paylen < sizeof(V2x_App_RxMsg)) return false;

  auto rx = parse_rx_container(payload, paylen);
  if (!rx.ok) return false;

  const uint8_t* cur    = rx.data;
  std::size_t    remain = rx.data_len;

  TlvcView ov{};
  if (!tlvc_next(cur, remain, ov) || (opts.strict_crc && !tlvc_check_crc_relaxed(ov)) || ov.type != EM_PT_OVERALL) {
    V2X_RXPARSER_WARN("Overall TLVC missing/invalid");
    return false;
  }

  uint8_t  num_pkg   = 0;
  uint16_t pkg_bytes = 0;
  if ((ov.total_len + 2) >= sizeof(TLVC_Overall_V2)) {
    const auto* o2 = reinterpret_cast<const TLVC_Overall_V2*>(ov.base);
    num_pkg   = o2->num_package;
    pkg_bytes = v2x::be16_from(&o2->len_package);
  } else if ((ov.total_len + 2) >= sizeof(TLVC_Overall)) {
    const auto* o1 = reinterpret_cast<const TLVC_Overall*>(ov.base);
    num_pkg   = o1->num_package;
    pkg_bytes = v2x::be16_from(&o1->len_package);
  } else {
    V2X_RXPARSER_WARN("Overall too short");
    return false;
  }

  const uint8_t* pkg_cur_begin = cur;
  std::size_t    pkg_remain_all = std::min<std::size_t>(remain, pkg_bytes);

  // 1) 프리스캔: SSOV 포함 여부
  bool has_ssov = false;
  if (opts.status_only_when_ssov) {
    const uint8_t* scan_cur = pkg_cur_begin;
    std::size_t    scan_rem = pkg_remain_all;
    for (uint8_t i=0; i<num_pkg && scan_rem>0; ++i) {
      TlvcView tv{};
      if (!tlvc_next(scan_cur, scan_rem, tv)) break;
      if (opts.strict_crc && !tlvc_check_crc_relaxed(tv)) continue;
      if (tv.type == EM_PT_SSOV) { has_ssov = true; break; }
    }
  }

  // 2) 실제 처리 루프
  const uint8_t* pkg_cur    = pkg_cur_begin;
  std::size_t    pkg_remain = pkg_remain_all;

  for (uint8_t i=0; i<num_pkg && pkg_remain>0; ++i) {
    TlvcView tv{};
    if (!tlvc_next(pkg_cur, pkg_remain, tv)) {
      V2X_RXPARSER_WARN("TLVC parse failed, remain=" << pkg_remain);
      break;
    }
    if (opts.strict_crc && !tlvc_check_crc_relaxed(tv)) {
      V2X_RXPARSER_WARN("TLVC CRC failed");
      continue;
    }

    switch (tv.type) {
      case EM_PT_VIDEO:
        if (handlers.on_video) handlers.on_video(tv);
        break;

      case EM_PT_RAW_DATA:
        if (handlers.on_raw) handlers.on_raw(tv.type, tv.v, tv.v_len);
        break;

      case EM_PT_STATUS: {
        if (!opts.status_only_when_ssov || has_ssov) {
          const uint8_t dev_type = *(tv.v + 0);
          const uint8_t tx_rx    = *(tv.v + 1);

          // Modem Tx
          if ((dev_type == eStatusDevType_ObuModem || dev_type == eStatusDevType_RsuModem) && tx_rx == eStatusTxRx_Tx) {
            if ((tv.total_len + 2) >= sizeof(TLVC_STATUS_Tx_ModemUnit_V2)){
              const auto* s = reinterpret_cast<const TLVC_STATUS_Tx_ModemUnit_V2*>(tv.base);
              if (handlers.on_status_tx_modem_v2) handlers.on_status_tx_modem_v2(s);
            }
            else{
              const auto* s = reinterpret_cast<const TLVC_STATUS_Tx_ModemUnit*>(tv.base);
              if (handlers.on_status_tx_modem_v1) handlers.on_status_tx_modem_v1(s);
            }
          }

          // Modem Rx
          if ((dev_type == eStatusDevType_ObuModem || dev_type == eStatusDevType_RsuModem) && tx_rx == eStatusTxRx_Rx){
            if ((tv.total_len + 2) >= sizeof(TLVC_STATUS_Rx_ModemUnit_V2)){
              const auto* s = reinterpret_cast<const TLVC_STATUS_Rx_ModemUnit_V2*>(tv.base);
              if (handlers.on_status_rx_modem_v2) handlers.on_status_rx_modem_v2(s);
            }
            else{
              const auto* s = reinterpret_cast<const TLVC_STATUS_Rx_ModemUnit*>(tv.base);
              if (handlers.on_status_rx_modem_v1) handlers.on_status_rx_modem_v1(s);
            }
          }
          
          // Comm Tx
          if (dev_type == eStatusDevType_Obu || dev_type == eStatusDevType_Rsu){
            if ((tv.total_len + 2) >= sizeof(TLVC_STATUS_CommUnit_V2)){
              const auto* s = reinterpret_cast<const TLVC_STATUS_CommUnit_V2*>(tv.base);
              if (handlers.on_status_comm_unit_v2) handlers.on_status_comm_unit_v2(s);
            }
            else{
              const auto* s = reinterpret_cast<const TLVC_STATUS_CommUnit*>(tv.base);
              if (handlers.on_status_comm_unit_v1) handlers.on_status_comm_unit_v1(s);
            }
          }

          // Control Unit
          if (dev_type == eStatusDevType_RsuControl){
            if ((tv.total_len + 2) >= sizeof(TLVC_STATUS_ControlUnit_V2)){
              const auto* s = reinterpret_cast<const TLVC_STATUS_ControlUnit_V2*>(tv.base);
              if (handlers.on_status_ctrl_unit_v2) handlers.on_status_ctrl_unit_v2(s);
            }
            else{
              const auto* s = reinterpret_cast<const TLVC_STATUS_ControlUnit*>(tv.base);
              if (handlers.on_status_ctrl_unit_v1) handlers.on_status_ctrl_unit_v1(s);
            }
          }
        }
        break;
      }

      case EM_PT_SSOV: {
        common::DbV2xFixed meta{};
        const uint8_t* p=nullptr; size_t n=0;
        if (parse_ssov_tlvc(tv, meta, p, n)) {
          if (handlers.on_ssov) handlers.on_ssov(meta, p, n);
        }
        break;
      }

      case EM_PT_TOD:
        // TODO: 확장 필요 시 추가
        break;

      default:
        // 알 수 없는 TLVC → RAW로 전달
        if (handlers.on_raw) handlers.on_raw(tv.type, tv.v, tv.v_len);
        break;
    }
  }

  return true;
}

}} // ns v2x::proto
