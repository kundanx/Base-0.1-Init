#include "Robot_main.h"
#include "Robot_config.h"
#include "Robot.hpp"
#include "usart.h"

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    if (huart->Instance == robot.joystick.huart->Instance)
    {
        robot.joystick.RxCallback();
    }
    else if (huart->Instance == robot.deadWheel.huart->Instance)
    {
        if (robot.deadWheel.get_received_data(robot.odom_rx_data) == OK)
        {
            memcpy(&robot.odom, robot.odom_rx_data, 12);
            printf("odom:: %f %f %f\n", robot.odom.x*100.0f, robot.odom.y*100.0f, robot.odom.theta*180.0f/M_PI);
        }
    }
}