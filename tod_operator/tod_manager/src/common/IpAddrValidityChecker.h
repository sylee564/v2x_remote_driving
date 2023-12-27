#ifndef TOD_IP_ADDR_VALIDITY_CHECKER_H
#define TOD_IP_ADDR_VALIDITY_CHECKER_H

#include <string>

class IpAddrValidityChecker {
public:
    virtual bool validate(const std::string& ip_addr) = 0;
    virtual ~IpAddrValidityChecker() = default;
};

#endif //TOD_IP_ADDR_VALIDITY_CHECKER_H

