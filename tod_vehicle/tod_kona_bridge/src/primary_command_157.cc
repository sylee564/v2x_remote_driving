#include "primary_command_157.hpp"

int32_t Primarycommand157::ID = 0x157;

// public
Primarycommand157::Primarycommand157() { Reset(); }

void Primarycommand157::UpdateData(int steer_angle_target, float acc_dec_cmd, uint8_t checksum_157) {
  set_steer_angle_target(steer_angle_target);
  set_accel_dec_cmd(acc_dec_cmd);
  set_checksum_157(checksum_157);
}

void Primarycommand157::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Primarycommand157::get_data()
{
  return data;
}

void Primarycommand157::set_steer_angle_target(int steer_angle_target) {
  //steer_angle_target = ProtocolData::BoundedValue(-360, 360, steer_angle_target);
  int x = (steer_angle_target*10);
  uint8_t t = 0;
  uint8_t a = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[0] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[1] += to_set1.return_byte_t();
}

void Primarycommand157::set_accel_dec_cmd(float acc_dec_cmd) {
  if(acc_dec_cmd > 1.00f)
    acc_dec_cmd = 1.00f;
  if(acc_dec_cmd < -3.00f)
    acc_dec_cmd = -3.00f;

  int x = static_cast<int>(((floor(acc_dec_cmd*100)/100) + 10.23)*100);
  uint8_t t = 0;
  uint8_t a = 0;

  t = x & 0xFF;
  Byte to_set(a);
  to_set.set_value(t, 0, 8);
  data[3] += to_set.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[4] += to_set1.return_byte_t();
}


void Primarycommand157::set_checksum_157(uint8_t checksum_157) {
  int x = checksum_157;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
}

