#pragma once

#include "Byte.hpp"


class Steeringreport710
{
 public:
  static int32_t ID;
  Steeringreport710();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  uint8_t get_eps_control_board();
  uint8_t get_eps_en();
  uint8_t get_eps_control_status();
  uint8_t get_override_status();
  float get_steering_angle();
  float get_steer_drv_tq();
  float get_steer_out_tq();
  uint8_t get_eps_alive_cnt();

  uint8_t eps_control_board;
  uint8_t eps_en;
  uint8_t eps_control_status;
  uint8_t override_status;
  float steering_angle;
  float steer_drv_tq;
  float steer_out_tq;
  uint8_t eps_alive_cnt;

 private:
  uint8_t bytes[8];
  
  uint8_t decode_eps_control_board();
  uint8_t decode_eps_en();
  uint8_t decode_eps_control_status();
  uint8_t decode_override_status();

  float decode_steering_angle();
  float decode_steer_drv_tq();
  float decode_steer_out_tq();

  uint8_t decode_eps_alive_cnt();
};


