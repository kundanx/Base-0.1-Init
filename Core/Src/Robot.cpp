#include "Robot.hpp"
#include "Robot_config.h"
#include "usart.h"
#include "dma.h"
#include "tim.h"
#include "Robot.hpp"
#include "control_struct.h"
#include <stdio.h>


void Robot::init()
{
#ifdef __DEBUG_MODE__
    printf("robot init\n");
#endif

    deadMotor.init();
    joystick.Init();

#ifdef __IMPLEMENT_IR__
    ir.Init();
#endif

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
    // printJoystickData(joystickData);
    set_state_from_joystick_data(joystickData);

    // ir.UpdateJP();

    // uint setCount = 3;
    // if ((ir.RJPCount < setCount) && !ir.isRJP)
    // {
    //     if (ir.Follow())
    //     {
    //         base_twist = ir.GetTwist();
    //     }
    // }

    // if ((ir.RJPCount < setCount) && ir.isRJP)
    // {
    //     ir.move(0, PI/2);
    // }

    // if (ir.RJPCount == setCount)
    // {
    //     HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
    //     EMERGENCY_BREAK();
    // }
    // float sss = 1;

    // deadMotor.run();

    robot_loop = HAL_GetTick();
}

void Robot::set_state_from_joystick_data(const JoystickData &jdata)
{
#ifndef __IMPLEMENT_IR__

    if ((HAL_GetTick() - joystick.GetLastUpdateTick()) > 100)
    {
#ifdef __DEBUG_MODE__
        // printf("delayed data\n");
#endif
        deadMotor.base_twist = Twist(0, 0, 0);
        return;
    }

    const uint8_t d_band = 50;
    float v = 0, theta = deadMotor.base_twist.get_theta(), omega = 0;

    if (abs(jdata.l_hatx) < d_band && abs(jdata.l_haty) < d_band &&
        jdata.lt < d_band && jdata.rt < d_band)
    {
        deadMotor.base_twist = Twist(0, theta, 0);
    }

    if (((abs(jdata.l_haty) > d_band) || (abs(jdata.l_hatx) > d_band)))
    {
        float magnitude = sqrt(pow(jdata.l_haty, 2) + pow(jdata.l_hatx, 2));
        v = map<float>(magnitude, 0, 128, 0, MAXIMUM_VELOCITY);
        theta = atan2(jdata.l_haty, jdata.l_hatx);
    }

    float speed_factor = 1.0;
    if (jdata.button1 & _BV(B_LB))
    {
        speed_factor = 0.5;
    }

    if (jdata.lt > 30)
    {
        omega = map<float>(jdata.lt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
    }
    else if (jdata.rt > 30)
    {
        omega = -map<float>(jdata.rt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
    }

    if (jdata.button1 & _BV(B_UP))
    {
        // theta = PI / 2;
        // v = MAXIMUM_VELOCITY * speed_factor;

        deadMotor.linearMove(1.0, PI/2);
    }
    else if (jdata.button1 & _BV(B_DOWN))
    {
        // theta = -PI / 2;
        // v = MAXIMUM_VELOCITY * speed_factor;

        deadMotor.linearMove(1.0, -PI/2);
    }
    else if (jdata.button2 & _BV(B_LEFT))
    {
        // theta = PI;
        // v = MAXIMUM_VELOCITY * speed_factor;

        deadMotor.linearMove(1.0, PI);
    }
    else if (jdata.button2 & _BV(B_RIGHT))
    {
        // theta = 0;
        // v = MAXIMUM_VELOCITY * speed_factor;

        deadMotor.linearMove(1.0, 0);
    }
#endif

    if ((jdata.button2 & _BV(B_XBOX)) && (jdata.button1 & _BV(B_LB)))
    {
        isEmergencyBreak = true;
        EMERGENCY_BREAK();
    }

#ifndef __IMPLEMENT_IR__
    deadMotor.base_twist = Twist::from_v_theta_omega(v, theta, omega);
    // printf("v th w: %f %f %f\n", v, theta, omega);
#endif

    deadMotor.run();
    robot_loop = HAL_GetTick();
}

void Robot::EMERGENCY_BREAK()
{
    deadMotor.stop();

    while (1)
    {

        deadMotor.hardBreak();
    }
}

void Robot::printJoystickData(const JoystickData &jData)
{
    printf("Received Data: %u %u %u %u %d %d %d %d\n", jData.button1, jData.button2, jData.lt, jData.rt, jData.l_hatx, jData.l_haty, jData.r_hatx, jData.r_haty);
}