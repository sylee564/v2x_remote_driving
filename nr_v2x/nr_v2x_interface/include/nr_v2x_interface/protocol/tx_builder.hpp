#pragma once
#include <vector>
#include <cstdint>
#include <cstring>
#include <algorithm>

#include <nr_v2x_helper/byte_order.hpp>
#include <nr_v2x_helper/crc16.hpp>
#include "nr_v2x_interface/protocol/vendor_protocol.hpp"

namespace v2x { namespace proto { namespace wire {

enum class OuterCrcMode { HostAdds, DeviceAdds };

// ---- TLVC item(일반화) ----
struct TlvcItem {
  uint32_t        type{0};           // EM_PT_*
  const uint8_t*  data{nullptr};
  std::size_t     len{0};
};

// ---- 상태 TLVC 옵션 ----
struct StatusCommUnitArgs {
  uint8_t  dev_type = eStatusDevType_Obu;
  uint8_t  tx_rx    = eStatusTxRx_Tx;
  uint32_t dev_id   = 1;
  uint16_t hw_ver   = 0x0002;
  uint16_t sw_ver   = 0x0003;
  uint64_t timestamp_be64 = 0;       // 0이면 내부에서 KST 포맷 계산
  int8_t   cpu_temp = 50;
  int8_t   peri_temp= 50;
  bool     enabled  = false;         // true면 STATUS TLVC 추가
};

// ---- DB_V2X meta ----
struct DbV2xMeta {
  DB_V2X_DEVICE_TYPE_t            eDeviceType = DB_V2X_DEVICE_TYPE_UNKNOWN;
  DB_V2X_TELECOMMUNICATION_TYPE_t eTeleCommType = DB_V2X_TELECOMM_TYPE_UNKNOWN;
  uint32_t unDeviceId=0;
  uint64_t ulTimeStamp=0;
  DB_V2X_SERVICE_ID_t    eServiceId=DB_V2X_SERVICE_ID_UNKNOWN;
  DB_V2X_ACTION_TYPE_t   eActionType=DB_V2X_ACTION_TYPE_UNKNOWN;
  DB_V2X_REGION_ID_t     eRegionId=DB_V2X_REGION_ID_UNKNOWN;
  DB_V2X_PAYLOAD_TYPE_t  ePayloadType=DB_V2X_PAYLOAD_TYPE_UNKNOWN;
  DB_V2X_COMMUNCATION_ID_t eCommId=DB_V2X_COMM_ID_UNKNOWN;
  uint16_t usDbVer=1, usHwVer=0, usSwVer=0;
};

static constexpr std::size_t DBV2X_FIXED_LEN = 40; // bytes

inline std::size_t write_db_v2x_body(uint8_t* out, std::size_t cap,
                                     const DbV2xMeta& m,
                                     const uint8_t* payload, std::size_t payload_len,
                                     uint32_t payload_crc32 = 0)
{
  if (!out || cap < DBV2X_FIXED_LEN + payload_len) return 0;
  uint8_t* p = out;
  auto w16=[&](uint16_t v){ v2x::put_be16(p,v); p+=2; };
  auto w32=[&](uint32_t v){ v2x::put_be32(p,v); p+=4; };
  auto w64=[&](uint64_t v){ v2x::put_be64(p,v); p+=8; };

  w16(m.eDeviceType); w16(m.eTeleCommType);
  w32(m.unDeviceId);  w64(m.ulTimeStamp);
  w16(m.eServiceId);  w16(m.eActionType);
  w16(m.eRegionId);   w16(m.ePayloadType);
  w16(m.eCommId);     w16(m.usDbVer);
  w16(m.usHwVer);     w16(m.usSwVer);
  w32(static_cast<uint32_t>(payload_len));
  w32(payload_crc32);

  if (payload_len) { std::memcpy(p, payload, payload_len); p += payload_len; }
  return static_cast<std::size_t>(p - out);
}

// ---- TLVC + CRC 작성 ----
inline std::size_t write_tlvc_and_crc(V2x_App_Ext_TLVC* tlvc, uint32_t type,
                                      const uint8_t* v_ptr, std::size_t v_len,
                                      v2x::crc16::Variant var)
{
  v2x::put_be32(reinterpret_cast<uint8_t*>(&tlvc->type), type);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&tlvc->len), static_cast<uint16_t>(v_len + 2));
  if (v_len && v_ptr) std::memcpy(tlvc->data, v_ptr, v_len);

  const uint16_t crc = v2x::crc16::compute(reinterpret_cast<const uint8_t*>(tlvc),
                                           static_cast<uint32_t>(6 + v_len),
                                           var);
  v2x::put_be16(reinterpret_cast<uint8_t*>(tlvc) + 6 + v_len, crc);
  return 6 + v_len + 2; // TLVC total(T+L+V) + CRC(2)
}

// ---- KST timestamp(KETI 포맷, 10us 단위) ----
inline uint64_t make_kst_timestamp_keti_format() {
  using namespace std::chrono;
  auto now = system_clock::now();
  auto us  = duration_cast<microseconds>(now.time_since_epoch()).count();

  // UTC -> KST(+9h)
  std::time_t sec = static_cast<time_t>(us / 1'000'000) + 9 * 3600;
  std::tm tm{}; gmtime_r(&sec, &tm);

  const uint64_t y = tm.tm_year + 1900;
  const uint64_t m = tm.tm_mon + 1;
  const uint64_t d = tm.tm_mday;
  const uint64_t H = tm.tm_hour;
  const uint64_t M = tm.tm_min;
  const uint64_t S = tm.tm_sec;
  const uint64_t us_only = static_cast<uint64_t>(us % 1'000'000);

  return
    y * 1000000000000000ULL +
    m *   10000000000000ULL +
    d *     100000000000ULL +
    H *       1000000000ULL +
    M *         10000000ULL +
    S *           100000ULL +
    (us_only / 10ULL);
}

// ---- WSR 프레임(그대로 유지) ----
inline std::vector<uint8_t> build_wsr_frame(uint16_t seq,
                                            uint8_t action, uint32_t psid,
                                            v2x::crc16::Variant crc_variant,
                                            v2x::proto::wire::OuterCrcMode outer_crc_mode)
{
  constexpr uint16_t PID = static_cast<uint16_t>(ePayloadId_WsmServiceReq);
  std::vector<uint8_t> buf(17, 0);

  auto* hdr = reinterpret_cast<V2x_App_Hdr*>(buf.data());
  std::memcpy(hdr->magic, V2X_INF_EXT_MAGIC, 4);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&hdr->seq), seq);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&hdr->payload_id), PID);

  auto* wsr = reinterpret_cast<V2x_App_WSR_Add_Crc*>(hdr->data);
  wsr->action = action;
  v2x::put_be32(reinterpret_cast<uint8_t*>(&wsr->psid), psid);


  const uint16_t body_len = static_cast<uint16_t>(2 + 2 + sizeof(V2x_App_WSR_Add_Crc));
  v2x::put_be16(reinterpret_cast<uint8_t*>(&hdr->len), body_len);

  const std::size_t total = 4 + 2 + body_len ;
  buf.resize(total);


  if (outer_crc_mode == v2x::proto::wire::OuterCrcMode::HostAdds) {
    const uint16_t tail = v2x::crc16::compute(buf.data() + 4,
                                              static_cast<uint32_t>(total - 6),
                                              crc_variant);
    v2x::put_be16(reinterpret_cast<uint8_t*>(&wsr->crc), tail);
  } else {
    v2x::put_be16(reinterpret_cast<uint8_t*>(&wsr->crc), 0x0000);
  }
  return buf;
}

}}} // ns wire

// ============================= v1 =============================
namespace v2x { namespace proto { namespace v1 {

using wire::TlvcItem;
using wire::OuterCrcMode;
using wire::DbV2xMeta;
using wire::StatusCommUnitArgs;
using wire::DBV2X_FIXED_LEN;
using wire::write_db_v2x_body;
using wire::write_tlvc_and_crc;
using wire::make_kst_timestamp_keti_format;

// v1 Overall에는 bitwize 없음
struct BuildArgs {
  uint16_t seq{};
  uint32_t psid{};
  uint8_t  castMode{};
  uint32_t srcId{}, dstId{};
  v2x::crc16::Variant crc_variant{v2x::crc16::Variant::XMODEM};
  OuterCrcMode outer_crc_mode{OuterCrcMode::DeviceAdds};

  // TLVC items + 소유 버퍼(수명 보장)
  std::vector<TlvcItem>              items;
  std::vector<std::vector<uint8_t>>  owned_blobs;

  // 상태 TLVC 옵션(v1에선 타입은 동일하게 EM_PT_STATUS 사용)
  StatusCommUnitArgs status{};
};

// SSOV를 쉽게 추가하기 위한 헬퍼
inline void add_ssov_item(BuildArgs& a, const DbV2xMeta& meta,
                          const uint8_t* payload, std::size_t payload_len,
                          bool take_ownership=true)
{
  std::vector<uint8_t> vbuf;
  vbuf.resize(DBV2X_FIXED_LEN + payload_len);
  const auto used = write_db_v2x_body(vbuf.data(), vbuf.size(), meta, payload, payload_len, 0);
  vbuf.resize(used);

  if (take_ownership) {
    a.owned_blobs.emplace_back(std::move(vbuf));
    const auto& ref = a.owned_blobs.back();
    a.items.push_back({EM_PT_SSOV, ref.data(), ref.size()});
  } else {
    a.items.push_back({EM_PT_SSOV, vbuf.data(), vbuf.size()}); // 주의: 외부 수명 보장 필요
  }
}

inline void add_tlvc_item(BuildArgs& a, uint32_t type, const uint8_t* ptr, std::size_t len,
                          bool take_ownership=true)
{
  if (!ptr || len==0) return;
  if (take_ownership) {
    a.owned_blobs.emplace_back(ptr, ptr + len);
    const auto& ref = a.owned_blobs.back();
    a.items.push_back({type, ref.data(), ref.size()});
  } else {
    a.items.push_back({type, ptr, len});
  }
}

inline std::vector<uint8_t> build_tx_extensible(const BuildArgs& a)
{
  std::vector<uint8_t> buf(64*1024, 0);
  auto* hdr   = reinterpret_cast<V2x_App_Hdr*>(buf.data());
  auto* txmsg = reinterpret_cast<V2x_App_TxMsg*>(hdr->data);

  // Overall(v1)
  auto* ov = reinterpret_cast<TLVC_Overall*>(txmsg->data);
  v2x::put_be32(reinterpret_cast<uint8_t*>(&ov->type), EM_PT_OVERALL);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&ov->len),  static_cast<uint16_t>(sizeof(TLVC_Overall) - 6));
  std::memcpy(ov->magic,"EMOP",4);
  ov->version    = 1;
  ov->num_package= 0;
  v2x::put_be16(reinterpret_cast<uint8_t*>(&ov->len_package), 0);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&ov->crc), 0);

  uint8_t*  p       = reinterpret_cast<uint8_t*>(ov) + sizeof(TLVC_Overall);
  uint16_t  pkg_len = 0;

  // (1) 사용자가 넣은 TLVC 목록
  for (const auto& it : a.items) {
    if (!it.data || it.len==0) continue;
    auto* tlvc = reinterpret_cast<V2x_App_Ext_TLVC*>(p);
    const std::size_t tlen = write_tlvc_and_crc(tlvc, it.type, it.data, it.len, a.crc_variant);
    p       += tlen;
    pkg_len += static_cast<uint16_t>(tlen);
    ov->num_package = static_cast<uint8_t>(ov->num_package + 1);
  }

  // (2) 옵션 STATUS
  if (a.status.enabled) {
    uint8_t v_status[1+1+4+2+2+8+1+1];
    uint8_t* q = v_status;
    *q++ = a.status.dev_type;
    *q++ = a.status.tx_rx;
    v2x::put_be32(q, a.status.dev_id); q+=4;
    v2x::put_be16(q, a.status.hw_ver); q+=2;
    v2x::put_be16(q, a.status.sw_ver); q+=2;

    uint64_t ts = (a.status.timestamp_be64!=0)?
                  a.status.timestamp_be64 : v2x::to_be64(make_kst_timestamp_keti_format());
    std::memcpy(q, &ts, 8); q+=8;

    *q++ = static_cast<uint8_t>(a.status.cpu_temp);
    *q++ = static_cast<uint8_t>(a.status.peri_temp);

    auto* tlvc1 = reinterpret_cast<V2x_App_Ext_TLVC*>(p);
    const std::size_t tlen1 = write_tlvc_and_crc(tlvc1, EM_PT_STATUS, v_status, sizeof(v_status), a.crc_variant);
    p       += tlen1;
    pkg_len += static_cast<uint16_t>(tlen1);
    ov->num_package = static_cast<uint8_t>(ov->num_package + 1);
  }

  // Overall finalize
  v2x::put_be16(reinterpret_cast<uint8_t*>(&ov->len_package), pkg_len);
  const uint16_t ov_crc = v2x::crc16::compute(reinterpret_cast<const uint8_t*>(ov),
                                              static_cast<uint32_t>(sizeof(TLVC_Overall) - 2),
                                              a.crc_variant);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&ov->crc), ov_crc);

  const std::size_t overall_total = sizeof(TLVC_Overall) + pkg_len;

  // Frame header
  std::memcpy(hdr->magic, V2X_INF_EXT_MAGIC, 4);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&hdr->seq), a.seq);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&hdr->payload_id), ePayloadId_TxMsg);

  v2x::put_be32(reinterpret_cast<uint8_t*>(&txmsg->psid), a.psid);
  txmsg->castMode = a.castMode;
  v2x::put_be32(reinterpret_cast<uint8_t*>(&txmsg->srcId), a.srcId);
  v2x::put_be32(reinterpret_cast<uint8_t*>(&txmsg->dstId), a.dstId);
  std::memset(txmsg->reserved, 0, sizeof(txmsg->reserved));

  // len = seq(2)+payload_id(2)+TxMsg+Overall(+TLVC들)
  const uint16_t body_len = static_cast<uint16_t>(2 + 2 + sizeof(V2x_App_TxMsg) + overall_total);
  v2x::put_be16(reinterpret_cast<uint8_t*>(&hdr->len), body_len);

  // total = 4(magic)+2(len)+body_len+2(tail)
  const std::size_t total = 4 + 2 + body_len + 2;
  buf.resize(total);

  uint8_t* tailp = buf.data() + total - 2;
  if (a.outer_crc_mode == OuterCrcMode::HostAdds) {
    const uint16_t tail = v2x::crc16::compute(buf.data() + 4,
                                              static_cast<uint32_t>(total - 6),
                                              a.crc_variant);
    v2x::put_be16(tailp, tail);
  } else {
    v2x::put_be16(tailp, 0x0000);
  }
  return buf;
}


}}} // ns v1

// ============================= v2 =============================
namespace v2x { namespace proto { namespace v2 {

using wire::TlvcItem;
using wire::OuterCrcMode;
using wire::DbV2xMeta;
using wire::StatusCommUnitArgs;
using wire::DBV2X_FIXED_LEN;
using wire::write_db_v2x_body;
using wire::write_tlvc_and_crc;
using wire::make_kst_timestamp_keti_format;

struct BuildArgs {
  uint16_t seq{};
  uint32_t psid{};
  uint8_t  castMode{};
  uint32_t srcId{}, dstId{};
  uint8_t  overall_bitwize{0};  // v2 only
  v2x::crc16::Variant crc_variant{v2x::crc16::Variant::XMODEM};
  OuterCrcMode outer_crc_mode{OuterCrcMode::DeviceAdds};

  std::vector<TlvcItem>              items;
  std::vector<std::vector<uint8_t>>  owned_blobs;

  StatusCommUnitArgs status{};
};

inline void add_ssov_item(BuildArgs& a, const DbV2xMeta& meta,
                          const uint8_t* payload, std::size_t payload_len,
                          bool take_ownership=true)
{
  std::vector<uint8_t> vbuf(DBV2X_FIXED_LEN + payload_len);
  const auto used = write_db_v2x_body(vbuf.data(), vbuf.size(), meta, payload, payload_len, 0);
  vbuf.resize(used);

  if (take_ownership) {
    a.owned_blobs.emplace_back(std::move(vbuf));
    const auto& ref = a.owned_blobs.back();
    a.items.push_back({EM_PT_SSOV, ref.data(), ref.size()});
  } else {
    // 외부 수명 보장 필요
    a.items.push_back({EM_PT_SSOV, vbuf.data(), vbuf.size()});
  }
}

inline void add_tlvc_item(BuildArgs& a, uint32_t type, const uint8_t* ptr, std::size_t len,
                          bool take_ownership=true)
{
  if (!ptr || len==0) return;
  if (take_ownership) {
    a.owned_blobs.emplace_back(ptr, ptr + len);
    const auto& ref = a.owned_blobs.back();
    a.items.push_back({type, ref.data(), ref.size()});
  } else {
    a.items.push_back({type, ptr, len});
  }
}

inline std::vector<uint8_t> build_tx_extensible(const BuildArgs& a)
{
  using namespace v2x;
  std::vector<uint8_t> buf(64*1024, 0);
  auto* hdr   = reinterpret_cast<V2x_App_Hdr*>(buf.data());
  auto* txmsg = reinterpret_cast<V2x_App_TxMsg*>(hdr->data);

  // Overall(v2)
  auto* ov = reinterpret_cast<TLVC_Overall_V2*>(txmsg->data);
  put_be32(reinterpret_cast<uint8_t*>(&ov->type), EM_PT_OVERALL);
  put_be16(reinterpret_cast<uint8_t*>(&ov->len),  static_cast<uint16_t>(sizeof(TLVC_Overall_V2) - 6));
  std::memcpy(ov->magic,"EMOP",4);
  ov->version     = 2;
  ov->num_package = 0;
  put_be16(reinterpret_cast<uint8_t*>(&ov->len_package), 0);
  ov->bitwize     = a.overall_bitwize;
  put_be16(reinterpret_cast<uint8_t*>(&ov->crc), 0);

  uint8_t* p       = reinterpret_cast<uint8_t*>(ov) + sizeof(TLVC_Overall_V2);
  uint16_t pkg_len = 0;

  // (1) 사용자가 넣은 TLVC들
  for (const auto& it : a.items) {
    if (!it.data || it.len==0) continue;
    auto* tlvc = reinterpret_cast<V2x_App_Ext_TLVC*>(p);
    const std::size_t tlen = write_tlvc_and_crc(tlvc, it.type, it.data, it.len, a.crc_variant);
    p       += tlen;
    pkg_len += static_cast<uint16_t>(tlen);
    ov->num_package = static_cast<uint8_t>(ov->num_package + 1);
  }

  // (2) 옵션 STATUS
  if (a.status.enabled) {
    uint8_t v_status[1+1+4+2+2+8+1+1];
    uint8_t* q = v_status;
    *q++ = a.status.dev_type;
    *q++ = a.status.tx_rx;
    put_be32(q, a.status.dev_id); q+=4;
    put_be16(q, a.status.hw_ver); q+=2;
    put_be16(q, a.status.sw_ver); q+=2;

    uint64_t ts = (a.status.timestamp_be64!=0)?
                  a.status.timestamp_be64 : to_be64(make_kst_timestamp_keti_format());
    std::memcpy(q, &ts, 8); q+=8;

    *q++ = static_cast<uint8_t>(a.status.cpu_temp);
    *q++ = static_cast<uint8_t>(a.status.peri_temp);

    auto* tlvc1 = reinterpret_cast<V2x_App_Ext_TLVC*>(p);
    const std::size_t tlen1 = write_tlvc_and_crc(tlvc1, EM_PT_STATUS, v_status, sizeof(v_status), a.crc_variant);
    p       += tlen1;
    pkg_len += static_cast<uint16_t>(tlen1);
    ov->num_package = static_cast<uint8_t>(ov->num_package + 1);
  }

  // Overall finalize
  put_be16(reinterpret_cast<uint8_t*>(&ov->len_package), pkg_len);
  const uint16_t ov_crc = v2x::crc16::compute(reinterpret_cast<const uint8_t*>(ov),
                                              static_cast<uint32_t>(sizeof(TLVC_Overall_V2) - 2),
                                              a.crc_variant);
  put_be16(reinterpret_cast<uint8_t*>(&ov->crc), ov_crc);

  const std::size_t overall_total = sizeof(TLVC_Overall_V2) + pkg_len;

  // Frame header
  std::memcpy(hdr->magic, V2X_INF_EXT_MAGIC, 4);
  put_be16(reinterpret_cast<uint8_t*>(&hdr->seq), a.seq);
  put_be16(reinterpret_cast<uint8_t*>(&hdr->payload_id), ePayloadId_TxMsg);

  put_be32(reinterpret_cast<uint8_t*>(&txmsg->psid), a.psid);
  txmsg->castMode = a.castMode;
  put_be32(reinterpret_cast<uint8_t*>(&txmsg->srcId), a.srcId);
  put_be32(reinterpret_cast<uint8_t*>(&txmsg->dstId), a.dstId);
  std::memset(txmsg->reserved, 0, sizeof(txmsg->reserved));

  // body_len은 tail(2B) 미포함 !!!
  const uint16_t body_len = static_cast<uint16_t>(2 + 2 + sizeof(V2x_App_TxMsg) + overall_total+2);
  put_be16(reinterpret_cast<uint8_t*>(&hdr->len), body_len);

  // total = magic4 + len2 + body_len + tail2
  const std::size_t total = static_cast<std::size_t>(body_len) + 6;
  // buf.resize(total);

  // tail CRC
  uint8_t* tailp = reinterpret_cast<uint8_t*>(hdr) + total - 2;
  if (a.outer_crc_mode == OuterCrcMode::HostAdds) {
    const uint16_t tail = v2x::crc16::compute(buf.data() + 4,
                                              static_cast<uint32_t>(total - 6),
                                              a.crc_variant);
    put_be16(tailp, tail);
  } else {
    put_be16(tailp, 0x0000);
  }
  buf.resize(total);
  return buf;
}


}}} // ns v2
