#include "IPv4ValidityChecker.h"

IPv4ValidityChecker::IPv4ValidityChecker() {}

bool IPv4ValidityChecker::validate(const std::string& ip_addr) {
    bool ret { false };
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ip_addr.c_str(), &(sa.sin_addr));
    if (result != 0) { ret = true; }
    return ret;
}
