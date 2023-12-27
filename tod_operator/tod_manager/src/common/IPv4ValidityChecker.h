#ifndef TOD_IPV4_VALIDITY_CHECKER_H
#define TOD_IPV4_VALIDITY_CHECKER_H

#include "IpAddrValidityChecker.h"
#include <string>
#include <arpa/inet.h>

class IPv4ValidityChecker : public IpAddrValidityChecker {
public:
    IPv4ValidityChecker();
    bool validate(const std::string& ip_addr) override;
};

#endif //TOD_IPV4_VALIDITY_CHECKER_H

