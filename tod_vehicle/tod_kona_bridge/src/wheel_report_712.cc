#include "wheel_report_712.hpp"

Wheelreport712::Wheelreport712() {}
int32_t Wheelreport712::ID = 0x712;

void Wheelreport712::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Wheelreport712::Parse(){
  wheel_spd_fl = decode_wheel_spd_fl();
  wheel_spd_fr = decode_wheel_spd_fr();
  wheel_spd_rl = decode_wheel_spd_rl();
  wheel_spd_rr = decode_wheel_spd_rr();
}

float Wheelreport712::decode_wheel_spd_fl() {
  Byte t0(*(bytes + 0));
  int32_t low = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t high = t1.get_byte(0, 8);
  high <<= 8;
  int32_t out = low | high;

  float ret = out * 0.03125;
  return ret;
}

float Wheelreport712::decode_wheel_spd_fr() {
  Byte t0(*(bytes + 2));
  int32_t low = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t high = t1.get_byte(0, 8);
  high <<= 8;
  int32_t out = low | high;

  float ret = out * 0.03125;
  return ret;
}

float Wheelreport712::decode_wheel_spd_rl() {
  Byte t0(*(bytes + 4));
  int32_t low = t0.get_byte(0, 8);

  Byte t1(*(bytes + 5));
  int32_t high = t1.get_byte(0, 8);
  high <<= 8;
  int32_t out = low | high;

  float ret = out * 0.03125;
  return ret;
}

float Wheelreport712::decode_wheel_spd_rr() {
  Byte t0(*(bytes + 6));
  int32_t low = t0.get_byte(0, 8);

  Byte t1(*(bytes + 7));
  int32_t high = t1.get_byte(0, 8);
  high <<= 8;
  int32_t out = low | high;

  float ret = out * 0.03125;
  return ret;
}