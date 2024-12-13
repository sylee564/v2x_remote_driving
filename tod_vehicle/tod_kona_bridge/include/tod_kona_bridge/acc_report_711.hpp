#pragma once
#include <iostream>
#include "Byte.hpp"

class Accreport711{
 public:
  static int32_t ID;
  Accreport711();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  uint8_t acc_en_status;
  uint8_t acc_control_board;
  uint8_t acc_user_can_err;
  uint8_t acc_veh_err;
  uint8_t acc_err;
  uint8_t acc_control_status;
  uint8_t vehicle_speed;
  float long_accel;
  uint8_t acc_alive_cnt;


 private:
  uint8_t bytes[8];

  uint8_t decode_acc_en_status();
  uint8_t decode_acc_control_board();
  uint8_t decode_acc_user_can_err();
  uint8_t decode_acc_veh_err();
  uint8_t decode_acc_err();
  uint8_t decode_acc_control_status();
  uint8_t decode_vehicle_speed();

  float decode_long_accel();

  uint8_t decode_acc_alive_cnt();

};

