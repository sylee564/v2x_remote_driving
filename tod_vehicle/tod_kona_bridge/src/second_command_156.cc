#include "second_command_156.hpp"

int32_t Secondcommand156::ID = 0x156;

// public
Secondcommand156::Secondcommand156() { Reset(); }

void Secondcommand156::UpdateData(uint8_t eps_en, uint8_t override, uint8_t eps_speed,
                  uint8_t acc_en, uint8_t aeb_en,  uint8_t indicator,
                  uint8_t gear_cmd, uint8_t checksum_156) {
  set_override_ignore(override);
  set_eps_en(eps_en);
  set_eps_speed(eps_speed); 
  set_aeb_en(aeb_en);
  set_acc_en(acc_en);

  set_gear_cmd(gear_cmd); 
  set_indicator_cmd(indicator);
  set_checksum_156(checksum_156);
}

void Secondcommand156::Reset() {
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Secondcommand156::get_data()
{
  return data;
}

void Secondcommand156::set_override_ignore(uint8_t override) {
  if(override == 1)
    data[0] += 4;
  else data[0] += 0;
}

void Secondcommand156::set_eps_en(uint8_t eps_en) {

  if(eps_en==1)
    data[0] += 1;
  else
    data[0] += 0;

}

void Secondcommand156::set_eps_speed(uint8_t eps_speed) {
  int x = eps_speed;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[1] += to_set.return_byte_t();
}

void Secondcommand156::set_aeb_en(uint8_t aeb_en) {
  data[2] += 0b01000000 & (aeb_en << 6);
}

void Secondcommand156::set_acc_en(uint8_t acc_en) {
  if(acc_en==1)
    data[2] += 1;
  else
    data[2] += 0;
}

void Secondcommand156::set_gear_cmd(uint8_t gear_cmd) {
  uint8_t x = gear_cmd;
  uint8_t t = 0;

  t = (x & 0xf) << 4;
  data[5] |= t;
}

void Secondcommand156::set_indicator_cmd(uint8_t indicator) {
  if(indicator == 1 ) data[5] += 2;
  else if(indicator  == 2) data[5] += 4;
  else if(indicator  == 3) data[5] += 1;
  else data[5] += 0;
}

void Secondcommand156::set_checksum_156(uint8_t checksum_156) {
  //checksum_103 = ProtocolData::BoundedValue(0, 255, checksum_103);
  int x = checksum_156;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
}