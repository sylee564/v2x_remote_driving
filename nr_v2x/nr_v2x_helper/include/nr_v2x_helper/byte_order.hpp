#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

#if defined(_WIN32)
  #include <winsock2.h>   // htons, htonl
  #include <ws2tcpip.h>
  #pragma comment(lib, "Ws2_32.lib")
#else
  #include <arpa/inet.h>  // htons, htonl
  #include <endian.h>
  #include <byteswap.h>
#endif

namespace v2x {

//------------------------
// 16/32: 표준 API 래핑 (Big-Endian 변환)
//------------------------
inline uint16_t to_be16(uint16_t v) { return htons(v); }
inline uint32_t to_be32(uint32_t v) { return htonl(v); }
inline uint16_t from_be16(uint16_t v){ return ntohs(v); }
inline uint32_t from_be32(uint32_t v){ return ntohl(v); }

//------------------------
// 64bit: 표준이 없어 직접 구현 (Big-Endian 변환)
//------------------------
#if defined(_WIN32)
  inline uint64_t bswap64(uint64_t x){
    return ((x & 0x00000000000000FFull) << 56) |
           ((x & 0x000000000000FF00ull) << 40) |
           ((x & 0x0000000000FF0000ull) << 24) |
           ((x & 0x00000000FF000000ull) << 8 ) |
           ((x & 0x000000FF00000000ull) >> 8 ) |
           ((x & 0x0000FF0000000000ull) >> 24) |
           ((x & 0x00FF000000000000ull) >> 40) |
           ((x & 0xFF00000000000000ull) >> 56);
  }
  inline uint64_t to_be64(uint64_t v){ return bswap64(v); }   // Windows는 LE 가정
  inline uint64_t from_be64(uint64_t v){ return bswap64(v); }
#else
  inline uint64_t to_be64(uint64_t v){
  #if __BYTE_ORDER == __LITTLE_ENDIAN
    return __bswap_64(v);
  #else
    return v;
  #endif
  }
  inline uint64_t from_be64(uint64_t v){
  #if __BYTE_ORDER == __LITTLE_ENDIAN
    return __bswap_64(v);
  #else
    return v;
  #endif
  }
#endif

//------------------------
// 바이트 배열 접근 유틸 (Big-Endian on wire)
//------------------------
inline void put_be16(uint8_t* p, uint16_t v){
  uint16_t be = to_be16(v);
  std::memcpy(p, &be, sizeof(be));
}
inline void put_be32(uint8_t* p, uint32_t v){
  uint32_t be = to_be32(v);
  std::memcpy(p, &be, sizeof(be));
}
inline void put_be64(uint8_t* p, uint64_t v){
  uint64_t be = to_be64(v);
  std::memcpy(p, &be, sizeof(be));
}

inline uint16_t get_be16(const uint8_t* p){
  uint16_t be; std::memcpy(&be, p, sizeof(be));
  return from_be16(be);
}
inline uint32_t get_be32(const uint8_t* p){
  uint32_t be; std::memcpy(&be, p, sizeof(be));
  return from_be32(be);
}
inline uint64_t get_be64(const uint8_t* p){
  uint64_t be; std::memcpy(&be, p, sizeof(be));
  return from_be64(be);
}

inline uint16_t be16_from(const void* p) { return get_be16(reinterpret_cast<const uint8_t*>(p)); }
inline uint32_t be32_from(const void* p) { return get_be32(reinterpret_cast<const uint8_t*>(p)); }
inline uint64_t be64_from(const void* p) { return get_be64(reinterpret_cast<const uint8_t*>(p)); }

//------------------------
// 바이트 배열 접근 유틸 (Little-Endian on wire)
// - 호스트 엔디안과 무관하게 바이트를 직접 조립/분해
//------------------------
inline void put_le16(uint8_t* p, uint16_t v){
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline void put_le32(uint8_t* p, uint32_t v){
  p[0] = static_cast<uint8_t>( v        & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8 ) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
inline void put_le64(uint8_t* p, uint64_t v){
  p[0] = static_cast<uint8_t>( v         & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8 )  & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16 ) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24 ) & 0xFF);
  p[4] = static_cast<uint8_t>((v >> 32 ) & 0xFF);
  p[5] = static_cast<uint8_t>((v >> 40 ) & 0xFF);
  p[6] = static_cast<uint8_t>((v >> 48 ) & 0xFF);
  p[7] = static_cast<uint8_t>((v >> 56 ) & 0xFF);
}

inline uint16_t get_le16(const uint8_t* p){
  return static_cast<uint16_t>( uint16_t(p[0]) | (uint16_t(p[1])<<8) );
}
inline uint32_t get_le32(const uint8_t* p){
  return  uint32_t(p[0])
        | (uint32_t(p[1])<<8)
        | (uint32_t(p[2])<<16)
        | (uint32_t(p[3])<<24);
}
inline uint64_t get_le64(const uint8_t* p){
  return  uint64_t(p[0])
        | (uint64_t(p[1])<<8)
        | (uint64_t(p[2])<<16)
        | (uint64_t(p[3])<<24)
        | (uint64_t(p[4])<<32)
        | (uint64_t(p[5])<<40)
        | (uint64_t(p[6])<<48)
        | (uint64_t(p[7])<<56);
}

inline uint16_t le16_from(const void* p) { return get_le16(reinterpret_cast<const uint8_t*>(p)); }
inline uint32_t le32_from(const void* p) { return get_le32(reinterpret_cast<const uint8_t*>(p)); }
inline uint64_t le64_from(const void* p) { return get_le64(reinterpret_cast<const uint8_t*>(p)); }

} // namespace v2x
