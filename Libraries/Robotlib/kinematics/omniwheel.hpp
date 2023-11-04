#ifndef OMNIWHEEL_H_
#define OMNIWHEEL_H_

#include "math.h"

const float motor_offsets[4] = {M_PI_4,
                                3 * M_PI_4,
                                5.0 * M_PI_4,
                                7.0 * M_PI_4};

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
};

class OmniwheelKinematics
{

public:
  float inverseCouplingMatrix[4][3];
  float base_diameter, wheel_diameter;

  OmniwheelKinematics() {}

  OmniwheelKinematics(const float _base_diameter, const float _wheel_diameter)
      : base_diameter(_base_diameter), wheel_diameter(_wheel_diameter)
  {
    for (int i = 0; i < 4; i++)
    {
      inverseCouplingMatrix[i][0] = sin(motor_offsets[i]);
      inverseCouplingMatrix[i][1] = -cos(motor_offsets[i]);
      inverseCouplingMatrix[i][2] = -base_diameter / 2.0;
    }
  }

  void get_motor_omega(const Twist _twist, float *_motor_omega)
  {
    for (int i = 0; i < 4; i++)
    {
      _motor_omega[i] = inverseCouplingMatrix[i][0] * _twist.vx;
      _motor_omega[i] += inverseCouplingMatrix[i][1] * _twist.vy;
      _motor_omega[i] += inverseCouplingMatrix[i][2] * _twist.w;
      _motor_omega[i] *= 2.0 / wheel_diameter;
    }
  }
};

#endif // OMNIWHEEL_H_
