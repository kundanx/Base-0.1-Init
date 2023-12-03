#ifndef __ROBOT
#define __ROBOT

// #define __IMPLEMENT_IR__

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "Robotlib/communication/uart_hs.hpp"
#include "Robotlib/actuators/DeadMotor.hpp"
#include "Robotlib/sensors/ir.hpp"
#include "definition.h"
#include "Robot_config.h"


class Robot
{
    public:
        Robot() {};  
        ~Robot() {};

        void init();
        void run();

        DeadMotor deadMotor;
        UartHS joystick{&JOYSTICK_UART};

#ifdef __IMPLEMENT_IR__
        IR ir{&IR_UART};
#endif

        uint32_t robot_loop = 0;

    private:
        void set_state_from_joystick_data(const JoystickData &joytick_data);
        void EMERGENCY_BREAK();
        void printJoystickData(const JoystickData& jdata);


        JoystickData joystickData;
        bool isEmergencyBreak = false;
};

#endif // __ROBOT