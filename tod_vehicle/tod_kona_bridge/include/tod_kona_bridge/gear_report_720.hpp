#pragma once

#include <iostream>
#include <Byte.hpp>

class Gearreport720{ 
 public:
  static int32_t ID;
  Gearreport720();
  void update_bytes(uint8_t bytes_data[8]);
  void Parse();
  uint8_t gear_cur;
  uint8_t gear_cmd;
  uint8_t gear_control_status;

 private:
  uint8_t bytes[8];

  uint8_t decode_gear_cur();
  uint8_t decode_gear_cmd();
  uint8_t decode_gear_control_status();

};



