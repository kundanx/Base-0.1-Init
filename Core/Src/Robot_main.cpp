#include "Robot_main.h"
#include "Robot_config.h"
#include "Robot.hpp"
#include "usart.h"

Robot robot;


int Robot_main()
{
    HAL_Delay(2000);
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
        if ((HAL_GetTick() - robot.robot_loop) < ROBOT_LOOP_TIME)
            continue;
        robot.run();

        robot.robot_loop = HAL_GetTick();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_FLUSH_DRREGISTER(huart);

    if (huart->Instance == robot.joystick.huart->Instance)
    {
        robot.joystick.RxCallback();
    }

#ifdef __IMPLEMENT_IR__
    else if (huart->Instance == robot.ir.huart->Instance)
    {
        robot.ir.RxCallback();
    }
#endif
}