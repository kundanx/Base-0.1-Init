#include "Robot_main.h"
#include "Robot_config.h"
#include "Robot.hpp"
#include "usart.h"

// #define __COUNT__

Robot robot;

int Robot_main()
{
    init_robot();
    operate_robot();

    return 0;
}

void init_robot()
{
    HAL_Delay(20);
    robot.init();
}

void operate_robot()
{
    HAL_Delay(20);

    while (1)
    {
        if ((HAL_GetTick() - robot.motor_loop) < MOTOR_LOOP_TIME)
            continue;
        robot.run();

        robot.motor_loop = HAL_GetTick();
    }
}

#ifdef __COUNT__
int32_t odata[3];
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    if (huart->Instance == robot.joystick.huart->Instance)
    {
        robot.joystick.RxCallback();
    }
    else if (huart->Instance == robot.deadMotor.deadWheel.huart->Instance)
    {
        if (robot.deadMotor.deadWheel.get_received_data(robot.deadMotor.odom_rx_data) == OK)
        {
#ifndef __COUNT__
            memcpy(&robot.deadMotor.odom, robot.deadMotor.odom_rx_data, 12);
            // printf("odom:: %f %f %f\n", robot.deadMotor.odom.x * 100.0f, robot.deadMotor.odom.y * 100.0f, robot.deadMotor.odom.theta * 180.0f / M_PI);
#endif

#ifdef __COUNT__
            memcpy(odata, robot.deadMotor.odom_rx_data, 12);
            printf("odom:: %ld %ld %ld\n", odata[0], odata[1], odata[2]);
#endif
        }
    }
}