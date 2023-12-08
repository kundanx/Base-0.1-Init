#ifndef __DEAD_MOTOR_HPP__
#define __DEAD_MOTOR_HPP__

#include "stm32f4xx_hal.h"
#include "Robot_config.h"
#include "definition.h"
#include "Robotlib/actuators/motor.hpp"
#include "Robotlib/sensors/encoder.hpp"
#include "Robotlib/controllers/pid.hpp"
#include "Robotlib/controllers/fuzzy_pid.hpp"
#include "Robotlib/kinematics/omniwheel.hpp"
#include "Robotlib/communication/uart.h"

#define MOTOR_LOOP_TIME 10

#define DEFAULT_VELOCITY MAXIMUM_VELOCITY
#define DEFAULT_OMEGA MAXIMUM_OMEGA

struct Odometry
{
    float x;
    float y;
    float theta;
};

class DeadMotor
{
public:
    DeadMotor() = default;
    ~DeadMotor() = default;

    void init();
    void run();
    void maintainCoordinates();
    void changeCoordinates(float x, float y, float theta);
    void changeTheta(float newTheta);
    void move(float velocity, float theta, float omega = 0);
    void stop();
    void hardBreak();

    UART deadWheel{&DEADWHEEL_UART, 12, RECEIVING};
    Odometry odom{0, 0, 0};
    Odometry odom_setpoint{0, 0, 0};
    uint8_t odom_rx_data[12];

    Twist base_twist;
    Motor base_motors[4];
    uint32_t motor_loop = 0;

    PID base_motor_pid_controllers[4];
    PID theta_pid;
    PID x_pid;
    PID y_pid;

private:
    Encoder base_motor_encoders[4];
    OmniwheelKinematics omniwheel_kinematics{BASE_DIAMETER, WHEEL_DIAMETER};

    float motor_omegas[4] = {0, 0, 0, 0};

};

#endif