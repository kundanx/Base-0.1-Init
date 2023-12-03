#ifndef __CONTROL_STRUCTURE__
#define __CONTROL_STRUCTURE__

#include <math.h>
#include <stdint.h>


struct Twist
{
  float vx, vy, w;

  Twist(float _vx, float _vy, float _w)
      : vx(_vx), vy(_vy), w(_w) {}

  Twist() : Twist(0, 0, 0) {}

  static Twist from_v_theta_omega(const float v, const float theta, const float omega)
  {
    return Twist(v * cos(theta), v * sin(theta), omega);
  }

  float get_theta() const
  {
    return atan2(vy, vx);
  }

  void set_data_from_v_thata_omega(const float v, const float theta, const float omega)
  {
    vx = v * cos(theta);
    vy = v * sin(theta);
    w = omega;
  }
};

/******************************************************************************
 * Joystick data structure
 * 
 * There are ten bytes in a packet of control data.
 * First byte is start byte.
 * Last byte is error detection byte.
 * Between eigth bytes include followings:
 * 
 * Button byte-1: X Y A B Up Down lb RB
 * Button byte-2: Start Back XBox Left right Left-hat-press Right-hat-press 0
 * LT
 * RT
 * Left-hat-x
 * Left-hat-y
 * Right-hat-x
 * Right-hat-y
*******************************************************************************/
struct JoystickData
{
  uint8_t button1 = 0;
  uint8_t button2 = 0;
  uint8_t lt = 0;
  uint8_t rt = 0;
  int8_t l_hatx = 0;
  int8_t l_haty = 0;
  int8_t r_hatx = 0;
  int8_t r_haty = 0;
};



#endif // __CONTROL_STRUCTURE__