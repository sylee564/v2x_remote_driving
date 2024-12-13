#pragma once

#include "Byte.hpp"
#include <iostream>

class Brakereport713
{
 public:
  static int32_t ID;
  Brakereport713();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  float lat_accel;
  float yaw_rate;
  float brake_cylinder;

 private:
  uint8_t bytes[8];
  
  float decode_lat_accel();
  float decode_yaw_rate();
  float decode_brake_cylinder();

};


