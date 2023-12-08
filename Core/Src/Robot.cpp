#include "Robot.hpp"
#include "Robot_config.h"
#include "usart.h"
#include "dma.h"
#include "tim.h"
#include "Robot.hpp"
#include "Robotlib/filters/ramp.hpp"
#include <stdio.h>


void Robot::init()
{
#ifdef __DEBUG_MODE__
    printf("robot init\n");
#endif

    joystick.Init();
    deadMotor.init();

    robot_loop = HAL_GetTick();
}

void Robot::run()
{
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

#ifdef __DEBUG_MODE__
    // joystick.ShowData();
    // joystick.ShowPacket();
#endif

    joystick.GetData(joystickData);
    set_state_from_joystick_data(joystickData);

    deadMotor.maintainCoordinates();
    // printf("vxp:: %lf  ", deadMotor.x_pid.Output);
    // printf("setpoint:: %.2f  %.2f  %.2f    ", round2(deadMotor.odom_setpoint.x * 100.0f), round2(deadMotor.odom_setpoint.y * 100.0f), round2(deadMotor.odom_setpoint.theta * 180.0f / PI));
    // printf("odom:: %.2f  %.2f  %.2f\n", round2(deadMotor.odom.x * 100.0f), round2(deadMotor.odom.y * 100.0f), round2(deadMotor.odom.theta * 180.0f / PI));

    robot_loop = HAL_GetTick();
}

void Robot::set_state_from_joystick_data(const JoystickData &jdata)
{

    if ((HAL_GetTick() - joystick.GetLastUpdateTick()) > 100)
    {
#ifdef __DEBUG_MODE__
        // printf("delayed data\n");
#endif
        deadMotor.base_twist = Twist(0, 0, 0);
        return;
    }

    // const uint8_t d_band = 50;
    // float v = 0, theta = deadMotor.base_twist.get_theta(), omega = 0;

    // if (abs(jdata.l_hatx) < d_band && abs(jdata.l_haty) < d_band &&
    //     jdata.lt < d_band && jdata.rt < d_band)
    // {
    //     deadMotor.base_twist = Twist(0, theta, 0);
    // }

    // if (((abs(jdata.l_haty) > d_band) || (abs(jdata.l_hatx) > d_band)))
    // {
    //     float magnitude = sqrt(pow(jdata.l_haty, 2) + pow(jdata.l_hatx, 2));
    //     v = map<float>(magnitude, 0, 128, 0, MAXIMUM_VELOCITY);
    //     theta = atan2(jdata.l_haty, jdata.l_hatx);
    // }

    // float speed_factor = 1.0;
    // if (jdata.button1 & _BV(B_LB))
    // {
    //     speed_factor = 0.2;
    // }
    // if (jdata.lt > 30)
    // {
    //     omega = map<float>(jdata.lt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
    // }
    // else if (jdata.rt > 30)
    // {
    //     omega = -map<float>(jdata.rt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
    // }

    if ((HAL_GetTick() - last_button_tick) > 100)
    {
        if ((jdata.button1 & _BV(B_UP)) && (!(prev_button1 & _BV(B_UP))))
        {
            deadMotor.odom_setpoint.y += 1.0;
            // printf("Up::");
            // printf("UP\n");
        }
        else if ((jdata.button1 & _BV(B_DOWN)) && (!(prev_button1 & _BV(B_DOWN))))
        {
            deadMotor.odom_setpoint.y -= 1.0;

            // printf("Down::");
        }
        else if ((jdata.button2 & _BV(B_LEFT)) && (!(prev_button2 & _BV(B_LEFT))))
        {
            deadMotor.odom_setpoint.x -= 1.0;
            // printf("Left::");
        }
        else if ((jdata.button2 & _BV(B_RIGHT)) && (!(prev_button2 & _BV(B_RIGHT))))
        {
            deadMotor.odom_setpoint.x += 1.0;
            // printf("Right::");
        }
        else if ((jdata.button1 & _BV(B_X)) && (!(prev_button1 & _BV(B_X))))
        {
            deadMotor.changeTheta(deadMotor.odom_setpoint.theta + M_PI_2);
        }
        else if ((jdata.button1 & _BV(B_B)) && (!(prev_button1 & _BV(B_B))))
        {
            deadMotor.changeTheta(deadMotor.odom_setpoint.theta - M_PI_2);
        }

        prev_button1 = jdata.button1;
        prev_button2 = jdata.button2;
        last_button_tick = HAL_GetTick();
    }

    if ((jdata.button2 & _BV(B_XBOX)) && (jdata.button1 & _BV(B_LB)))
    {
        EMERGENCY_BREAK();
    }

    // deadMotor.base_twist = Twist::from_v_theta_omega(v, theta, omega);
    // printf("v th w: %f %f %f\n", v, theta, omega);
}

void Robot::EMERGENCY_BREAK()
{
    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            deadMotor.base_motors[i].set_speed(0.0);
        }
    }
}
