#pragma once

#include "Byte.hpp"


class Secondcommand156{
 public:
  static int32_t ID;

  Secondcommand156();

  void UpdateData(uint8_t eps_en, uint8_t override, uint8_t eps_speed,
                  uint8_t acc_en, uint8_t aeb_en,  uint8_t indicator,
                  uint8_t gear_cmd, uint8_t checksum_156);

  void Reset();
  
  uint8_t * get_data();

 private:

  void set_override_ignore(uint8_t override);
  void set_eps_en(uint8_t eps_en);
  void set_eps_speed(uint8_t eps_speed); 
  void set_aeb_en(uint8_t aeb_en);
  void set_acc_en(uint8_t acc_en);
  void set_gear_cmd(uint8_t gear_cmd); 
  void set_indicator_cmd(uint8_t indicator);
  void set_checksum_156(uint8_t checksum_156);

 private:
  uint8_t data[8];
};



