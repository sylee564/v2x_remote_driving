#pragma once

// 구조체 패킹 매크로(컴파일러별 호환)
#if defined(_MSC_VER)
  // MSVC: #pragma pack 사용
  #define V2X_PACKED_STRUCT(def) __pragma(pack(push, 1)) def __pragma(pack(pop))
  #define V2X_ALIGNED(N) __declspec(align(N))
#else
  // GCC/Clang: packed attribute
  #define V2X_PACKED_STRUCT(def) def __attribute__((__packed__))
  #define V2X_ALIGNED(N) __attribute__((aligned(N)))
#endif

// 예시)
// V2X_PACKED_STRUCT(
//   struct MyHdr {
//     uint32_t a;
//     uint16_t b;
//     uint8_t  c;
//   };
// );

// 정렬 강제 예시: V2X_ALIGNED(8) struct MyBuf { ... };
