#ifndef __ROBOT
#define __ROBOT

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "Robotlib/actuators/motor.hpp"
#include "Robotlib/sensors/encoder.hpp"
#include "Robotlib/controllers/pid.hpp"
#include "Robotlib/controllers/fuzzy_pid.hpp"
#include "Robotlib/kinematics/omniwheel.hpp"
#include "Robotlib/communication/uart_hs.hpp"
#include "Robotlib/communication/uart.h"

#include "definition.h"
#include "Robot_config.h"

#define MOTOR_LOOP_TIME 10
#define ROBOT_LOOP_TIME 10

struct Odometry
{
  float x;
  float y;
  float theta;
};

class Robot
{
    public:

        Robot() {};  
        ~Robot() {};

        void init();
        void run();

        UartHS joystick{&JOYSTICK_UART};
        UART deadWheel{&DEADWHEEL_UART, 12, RECEIVING};
        Odometry odom{0, 0, 0};

        uint8_t odom_rx_data[12];

        uint32_t motor_loop = 0;
        uint32_t robot_loop = 0;

    private:
        void set_state_from_joystick_data(const JoystickData &joytick_data);
        void EMERGENCY_BREAK();
        void printJoystickData(const JoystickData& jdata);

        Motor base_motors[4];
        Encoder base_motor_encoders[4];


        OmniwheelKinematics omniwheel_kinematics{BASE_DIAMETER, WHEEL_DIAMETER};
        PID base_motor_pid_controllers[4];
        Twist base_twist{0, 0, 0};
        JoystickData joystickData;

        float motor_omegas[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

#endif // __ROBOT