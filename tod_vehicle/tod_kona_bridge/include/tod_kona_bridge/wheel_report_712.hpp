#pragma once

#include "Byte.hpp"

class Wheelreport712{
 public:
  static int32_t ID;
  Wheelreport712();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  float wheel_spd_fl;
  float wheel_spd_fr;
  float wheel_spd_rl;
  float wheel_spd_rr;

 private:
  uint8_t bytes[8];

  float decode_wheel_spd_fl();
  float decode_wheel_spd_fr();
  float decode_wheel_spd_rl();
  float decode_wheel_spd_rr();
};