#pragma once

#include "Byte.hpp"
#include <math.h>

class Primarycommand157{
 public:
  static int32_t ID;

  Primarycommand157();

  void UpdateData(int steer_angle_target, float acc_dec_cmd, uint8_t checksum_157);

  void Reset();
  uint8_t * get_data();

 private:

  void set_steer_angle_target(int steer_angle_target);
  void set_accel_dec_cmd(float acc_dec_cmd);
  void set_checksum_157(uint8_t checksum_157);

 private:
  uint8_t data[8];
};


