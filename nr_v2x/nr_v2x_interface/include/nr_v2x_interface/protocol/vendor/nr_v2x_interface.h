#ifndef _NR_V2X_INTERFACE_H_
#define _NR_V2X_INTERFACE_H_
#pragma pack(1)

#include "v2x_app_ext.h"   // V2x_App_Hdr, V2x_App_TxMsg, V2x_App_RxMsg, V2x_App_Ext_TLVC

// ---- 기본 필드 크기 (wire) --------------------------------------------------
#define SIZE_MAGIC_NUMBER_OF_HEADER   (sizeof(uint32_t))  // "5GVX"
#define SIZE_LENTH_OF_HEADER          (sizeof(uint16_t))  // len (network byte order)
#define SIZE_SEQ_OF_HEADER            (sizeof(uint16_t))
#define SIZE_PID_OF_HEADER            (sizeof(uint16_t))
#define SIZE_CRC16_OF_TAIL            (sizeof(uint16_t))

// ---- 헤더/페이로드 고정부 크기 ----------------------------------------------
/*
 V2x_App_Hdr = magic(4) + len(2) + seq(2) + pid(2) = 10B (data[0] 제외)
 TxMsg fixed  = psid(4) + cast(1) + src(4) + dst(4) + reserved(8)            = 21B
 RxMsg fixed  = psid(4) + cast(1) + src(4) + dst(4) + reserved(8) + rcpi(1)  = 22B
 TLVC header  = type(4) + len(2)                                             =  6B  (len에는 V+CRC(2)가 포함됨)
*/
#define SIZE_V2X_APP_EXT_HEADER       (SIZE_MAGIC_NUMBER_OF_HEADER + SIZE_LENTH_OF_HEADER + SIZE_SEQ_OF_HEADER + SIZE_PID_OF_HEADER) // 10
#define SIZE_V2X_HEADER_WITHOUT_MAGIC (SIZE_V2X_APP_EXT_HEADER - SIZE_MAGIC_NUMBER_OF_HEADER) // 6

#define SIZE_TX_HEADER                (4 + 1 + 4 + 4 + 8)   // 21
#define SIZE_RX_HEADER                (4 + 1 + 4 + 4 + 8 + 1) // 22
#define SIZE_EXT_HEADER               (sizeof(V2x_App_Ext_TLVC)) // = 6 (type+len), data[0]이라 최소 헤더 크기

// ---- 전체 패킷 오버헤드(데이터 제외) -----------------------------------------
/* 총패킷 = 헤더(10) + 고정부 + 데이터 + CRC(2) */
#define SIZE_TX_PACKET_OVERHEAD       (SIZE_V2X_APP_EXT_HEADER + SIZE_TX_HEADER + SIZE_CRC16_OF_TAIL) // 10+21+2=33
#define SIZE_RX_PACKET_OVERHEAD       (SIZE_V2X_APP_EXT_HEADER + SIZE_RX_HEADER + SIZE_CRC16_OF_TAIL) // 10+22+2=34

// (구) 이름들은 오해의 소지가 있어 비권장. 필요하면 아래처럼 해석하세요.
// - "HDR_LEN_EXCEPT_DATA": 헤더+고정부+tail 중 '데이터 제외' 총합을 의미한다면 TX=33, RX=34가 맞습니다.
#define SIZE_HDR_LEN_EXCEPT_DATA_TX   (SIZE_TX_PACKET_OVERHEAD)   // DEPRECATED: 명시적 이름 사용 권장
#define SIZE_HDR_LEN_EXCEPT_DATA_RX   (SIZE_RX_PACKET_OVERHEAD)   // DEPRECATED
// - "CRC_LEN_EXCEPT_DATA": CRC가 커버하는 구간은 [len 필드~CRC 직전] = (total_len - 6) 바이트입니다.
//   고정 상수로 두기보다 런타임에 계산하세요(패킷 길이에 의존).

// ---- PSID / 페이로드 한계 ----------------------------------------------------
#define MAX_PSID_VALUE                (270549119U)
#define MAX_DATA_SIZE                 (8999)  // raw 메시지 최대
#define MAX_TX_PAYLOAD_TO_MODULE      (MAX_DATA_SIZE)
#define MAX_RX_PAYLOAD_BY_MODULE      (MAX_DATA_SIZE)

// ---- 최대 패킷 길이(버퍼 크기 산정) ------------------------------------------
#define MAX_TX_PACKET_TO_OBU          (MAX_DATA_SIZE + SIZE_TX_PACKET_OVERHEAD)
#define MAX_RX_PACKET_BY_OBU          (MAX_DATA_SIZE + SIZE_RX_PACKET_OVERHEAD)

// (오타 수정) 아래 매크로는 기존 코드에 남아 있던 참조 오류가 있어 제거/대체합니다.
// #define MAX_RX_LEN_FROM_DEV         (MAX_RX_PACKET_BY_DEVICE - SIZE_MAGIC_NUMBER_OF_HEADER - SIZE_LENTH_OF_HEADER)
// → 필요하다면 "len 필드 최대값"은 (총수신길이 - 6) 이므로:
#define MAX_RX_LEN_FIELD_FROM_OBU     (MAX_RX_PACKET_BY_OBU - (SIZE_MAGIC_NUMBER_OF_HEADER + SIZE_LENTH_OF_HEADER))


#ifdef __cplusplus
// 개발 중 크기 불일치가 생기면 즉시 컴파일 타임에 잡기 위한 체크(ROS C++ 빌드에만 적용)
static_assert(SIZE_V2X_APP_EXT_HEADER == 10, "V2x_App_Hdr must be 10 bytes (without data[0])");
static_assert(SIZE_TX_HEADER == 21,          "V2x_App_TxMsg fixed part must be 21 bytes");
static_assert(SIZE_RX_HEADER == 22,          "V2x_App_RxMsg fixed part must be 22 bytes");
#endif

#endif //_NR_V2X_INTERFACE_H_
