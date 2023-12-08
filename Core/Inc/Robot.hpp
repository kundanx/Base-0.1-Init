#ifndef __ROBOT
#define __ROBOT

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "Robotlib/actuators/DeadMotor.hpp"
#include "Robotlib/communication/uart_hs.hpp"
#include "Robotlib/communication/uart.h"
#include "definition.h"
#include "Robot_config.h"

class Robot
{
public:
    Robot(){};
    ~Robot(){};

    void init();
    void run();

    DeadMotor deadMotor;
    UartHS joystick{&JOYSTICK_UART};
    
    uint32_t motor_loop = 0;
    uint32_t robot_loop = 0;

private:
    void set_state_from_joystick_data(const JoystickData &joytick_data);
    void EMERGENCY_BREAK();
    void printJoystickData(const JoystickData &jdata);

    JoystickData joystickData;

    uint32_t last_button_tick = 0;
    uint32_t prev_button1 = 0;
    uint32_t prev_button2 = 0;
};

#endif // __ROBOT