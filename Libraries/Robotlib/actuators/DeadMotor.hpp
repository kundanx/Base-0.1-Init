#ifndef __DEAD_MOTOR_HPP__
#define __DEAD_MOTOR_HPP__

#include "stm32f4xx_hal.h"
#include "Robot_config.h"
#include "control_struct.h"
#include "definition.h"
#include "Robotlib/actuators/motor.hpp"
#include "Robotlib/sensors/encoder.hpp"
#include "Robotlib/controllers/pid.hpp"
#include "Robotlib/controllers/fuzzy_pid.hpp"
#include "Robotlib/kinematics/omniwheel.hpp"


#define DEFAULT_OMEGA       0.3*MAXIMUM_OMEGA
#define DEFAULT_VELOCITY    0.5*MAXIMUM_VELOCITY

class DeadMotor
{
public:
    DeadMotor() = default;
    ~DeadMotor() = default;

    void init();
    void run();
    void move(float velocity, float theta, float omega = 0);
    void linearMove(float disp, float theta, float velocity = DEFAULT_VELOCITY);
    void angularMove(float disp, float omega = DEFAULT_OMEGA);
    void stop();
    void hardBreak();

    Twist base_twist;
    uint32_t motor_loop = 0;

private:
    Motor base_motors[4];
    Encoder base_motor_encoders[4];
    OmniwheelKinematics omniwheel_kinematics{BASE_DIAMETER, WHEEL_DIAMETER};

    float motor_omegas[4] = {0, 0, 0, 0};


#ifdef _FUZZY_PID
    float base_motor_pid_inputs[4], base_motor_pid_outputs[4], base_motor_pid_setpoints[4];
#endif

#ifdef _CLASSICAL_PID
    PID base_motor_pid_controllers[4];
#endif
#ifdef _FUZZZY_PID
    fuzzy_pid base_motor_fuzzy_pid[4];
#endif
};

#endif