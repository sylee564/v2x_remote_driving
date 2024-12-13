#include "steering_report_710.hpp"

Steeringreport710::Steeringreport710() {}
int32_t Steeringreport710::ID = 0x710;

void Steeringreport710::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Steeringreport710::Parse(){
  eps_control_board = decode_eps_control_board();
  eps_en = decode_eps_en();
  eps_control_status = decode_eps_control_status();
  override_status = decode_override_status();
  steering_angle = decode_steering_angle();
  steer_drv_tq = decode_steer_drv_tq();
  steer_out_tq = decode_steer_out_tq();
  eps_alive_cnt = decode_eps_alive_cnt();
}

uint8_t Steeringreport710::decode_eps_control_board() {
  Byte t0(*(bytes + 0));
  uint8_t x = t0.get_byte(1, 3);

  uint8_t ret = x;
  return ret;
}

uint8_t Steeringreport710::decode_eps_en() {
  Byte t0(*(bytes + 0));
  uint8_t x = t0.get_byte(0, 1);

  uint8_t ret = x;
  return ret;
}

uint8_t Steeringreport710::decode_eps_control_status() {
  Byte t0(*(bytes + 1));
  uint8_t x = t0.get_byte(0, 4);

  uint8_t ret = x;
  return ret;
}

uint8_t Steeringreport710::decode_override_status() {
  Byte t0(*(bytes + 1));
  uint8_t x = t0.get_byte(5, 1);

  uint8_t ret = x;
  return ret;
}

float Steeringreport710::decode_steering_angle() {
  Byte t0(*(bytes + 2));
  int32_t low = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t high = t1.get_byte(0, 8);
  high <<= 8;
  int32_t out = low|high;
  
  if (out < 32768)
        out = out;
    else if (out > 32768)
        out -= 0xffff;

  float ret =  out*0.1;
  return ret;
}

float Steeringreport710::decode_steer_drv_tq() {
  Byte t0(*(bytes + 4));
  int32_t low = t0.get_byte(0, 8);
  
  Byte t1(*(bytes + 5));
  int32_t high = t0.get_byte(0, 4);
  high <<= 8;
  int32_t out  = low|high;
  int ret = (out-2048)*0.01;

  return ret;
}

float Steeringreport710::decode_steer_out_tq() {
  Byte t0(*(bytes + 5));
  int32_t low = t0.get_byte(4, 4);
  
  Byte t1(*(bytes + 6));
  int32_t high = t0.get_byte(0, 8);
  high <<= 4;
  int32_t out  = low|high;
  int ret = (out-2048)*0.01;
  
  return ret;
}

uint8_t Steeringreport710::decode_eps_alive_cnt() {
  Byte t0(*(bytes + 7));
  uint8_t x = t0.get_byte(0, 8);

  uint8_t ret = x;
  return ret;
}

uint8_t Steeringreport710::get_eps_control_board()
{
  return eps_control_board;
}

uint8_t Steeringreport710::get_eps_en()
{
  return eps_en;
}

uint8_t Steeringreport710::get_eps_control_status()
{
  return eps_control_status;
}

uint8_t Steeringreport710::get_override_status()
{
  return override_status;
}

float Steeringreport710::get_steering_angle()
{
  return steering_angle;
}

float Steeringreport710::get_steer_drv_tq()
{
  return steer_drv_tq;
}

float Steeringreport710::get_steer_out_tq()
{
  return steer_out_tq;
}

uint8_t Steeringreport710::get_eps_alive_cnt()
{
  return eps_alive_cnt;
}

