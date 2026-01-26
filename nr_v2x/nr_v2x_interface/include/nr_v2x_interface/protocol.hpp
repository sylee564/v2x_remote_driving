#ifndef V2X_INTERFACE_PROTOCOL_HPP
#define V2X_INTERFACE_PROTOCOL_HPP

#include <cstdint>
#include <string>
#include <algorithm>
#include <type_traits>  // <-- add
#include <cctype>       // <-- add

namespace v2x_interface {

enum class protocol_t : std::uint8_t { UDP = 0, TCP = 1 };

inline const char* to_cstr(protocol_t p) noexcept {
    switch (p) {
        case protocol_t::UDP: return "UDP";
        case protocol_t::TCP: return "TCP";
        default:              return "UNKNOWN";
    }
}

inline std::string to_string(protocol_t p) {
    return std::string(to_cstr(p));
}

[[nodiscard]] inline bool from_string(const std::string& s, protocol_t& out) {
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(),
                   [](unsigned char c){ return static_cast<char>(std::toupper(c)); }); // <-- fix
    if (t == "UDP") { out = protocol_t::UDP; return true; }
    if (t == "TCP") { out = protocol_t::TCP; return true; }
    return false;
}

template <class E>
constexpr auto to_underlying(E e) noexcept {
    return static_cast<std::underlying_type_t<E>>(e);
}

} // namespace v2x_interface
#endif // V2X_INTERFACE_PROTOCOL_HPP
