#include "ir.hpp"
#include "definition.h"
#include "gpio.h"
#include <stdio.h>
#include "Robot_config.h"

const float kp = 1.5, ki = 0, kd = 0;

const float pidOutputLimit = 100.0;

IR::IR(UART_HandleTypeDef *_huart)
    : huart(_huart) {}

void IR::Init()
{
    lastUpdateTick = HAL_GetTick();

    pid = PID(1.0, 0.0, 0.0, P_ON_E, REVERSE);
    pid.Init();
    pid.SetOutputLimits(-pidOutputLimit, pidOutputLimit);
    pid.SetTunings(kp, ki, kd);
    pid.SetSampleTime(ROBOT_LOOP_TIME * 2);
    pid.SetMode(AUTOMATIC);
    pid.Setpoint = 35;

    HAL_UART_Receive_IT(huart, &buffer, 1);
}

void IR::RxCallback()
{

    uint32_t now = HAL_GetTick();
    if ((now - lastUpdateTick) > 50)
    {
        HAL_GPIO_TogglePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin);
        lastUpdateTick = now;
        // printf("%u\n", buffer);
    }

    HAL_UART_Receive_IT(huart, &buffer, 1);
}

void IR::UpdateJP()
{

    // printf("%d %d %d %d %d\n",
    // HAL_GPIO_ReadPin(IR_JPLL_PORT, IR_JPLL_PIN),
    // HAL_GPIO_ReadPin(IR_JPLR_PORT, IR_JPLR_PIN),
    // HAL_GPIO_ReadPin(IR_JPC_PORT, IR_JPC_PIN),
    // HAL_GPIO_ReadPin(IR_JPRL_PORT, IR_JPRL_PIN),
    // HAL_GPIO_ReadPin(IR_JPRR_PORT, IR_JPRR_PIN));

    // HAL_Delay(100);


    if ((buffer >= 0) && (buffer <= 70))
    {
        
        bool curJPC = HAL_GPIO_ReadPin(IR_JPC_PORT, IR_JPC_PIN);
        bool curJPLL = HAL_GPIO_ReadPin(IR_JPLL_PORT, IR_JPLL_PIN);
        bool curJPLR = HAL_GPIO_ReadPin(IR_JPLR_PORT, IR_JPLR_PIN);
        bool curJPRL = HAL_GPIO_ReadPin(IR_JPRL_PORT, IR_JPRL_PIN);
        bool curJPRR = HAL_GPIO_ReadPin(IR_JPRR_PORT, IR_JPRR_PIN);

        if (curJPC && curJPLL && curJPLR && curJPRL && curJPRR)
        {
            if (!isCJP)
            {
                isCJP = true;
                ++CJPCount;
            }
        }
        else
        {
            isCJP = false;
        }

        if (abs(buffer - 35) <= 25)
        {
            if (curJPLL && curJPLR)
            {
                if (!isLJP)
                {
                    isLJP = true;
                    ++LJPCount;
                }
            }
            else
            {
                isLJP = false;
            }

            if (curJPRL && curJPRR)
            {
                if (!isRJP)
                {
                    isRJP = true;
                    ++RJPCount;
                    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
                }
            }
            else
            {
                isRJP = false;
            }
        }
    }

    printf("JP L C R:: %d %d %d,  JPCount L C R:: %ld %ld %ld\n",
           (int8_t)isLJP,
           (int8_t)isCJP,
           (int8_t)isRJP,
           LJPCount,
           CJPCount,
           RJPCount);
}

bool IR::Follow()
{
    // printf("Follow\n");
    // v = 0;
    // theta = PI / 2;
    // omega = 0.5 * MAXIMUM_OMEGA;
    // twist.set_data_from_v_thata_omega(v, theta, omega);
    // return true;

    ///////////////////////////////////
    rawData = buffer;

    v = 0.5 * MAXIMUM_VELOCITY;
    theta = PI / 2;

    if ((rawData >= 0) && (rawData <= 70))
    {
        pid.Input = rawData;
        if (rawData < 35)
        {
            mem = MEM_LEFT;
        }
        else if (rawData > 35)
        {
            mem = MEM_RIGHT;
        }
        else
        {
            mem = MEM_CENTER;
        }
    }
    else if (rawData == 255)
    {
        if (mem == MEM_LEFT)
        {
            v = 0.3 * MAXIMUM_VELOCITY;
            pid.Input = 0;
        }
        else if (mem == MEM_RIGHT)
        {
            v = 0.3 * MAXIMUM_VELOCITY;
            pid.Input = 70;
        }
        else
        {
            v = 0.3 * MAXIMUM_VELOCITY;
            theta = 3 * PI / 2;
            pid.Input = 35;
        }
    }

    if (pid.Compute())
    {
        omega = (-pid.Output / pidOutputLimit) * MAXIMUM_OMEGA * 0.3;
        twist.set_data_from_v_thata_omega(v, theta, omega);
        // printf("%u %f %f %f\n", rawData, v, theta, omega);

        return true;
    }

    return false;
}

// void IR::Compute()
// {
//     rawData = buffer;
//     theta = PI / 2;

//     if ((rawData >= 25) && (rawData <= 45))
//     {
//         v = ComputeV(0.5);
//         omega = 0;
//         mem = MEM_CENTER;
//     }
//     else if ((rawData >=10) && (rawData <= 60))
//     {
//         v = ComputeV(0.5);
//         omega = OmegaFromRawData();
//         mem = MEM_CENTER;
//     }
//     else if ((rawData >= 0) && (rawData <= 70))
//     {
//         v = ComputeV(0.5);
//         if (rawData < 35)
//         {
//             omega = OmegaFromRawData();
//             mem = MEM_LEFT;
//         }
//         else
//         {
//             omega = OmegaFromRawData();
//             mem = MEM_RIGHT;
//         }
//     }
//     else if (rawData == 255)
//     {
//         v = 0.5;
//         if (mem == MEM_LEFT)
//         {
//             omega = ComputeOmega(1.0);
//         }
//         else if (mem == MEM_RIGHT)
//         {
//             omega = ComputeOmega(-1.0);
//         }
//         else
//         {
//             v = ComputeV(0.2);
//             theta = 3 * PI / 2;
//         }

//     }

//     twist.set_data_from_v_thata_omega(v, theta, omega);
//     // printf("%u %f %f %f\n", rawData, v, theta, omega);
// }

Twist IR::GetTwist()
{
    // printf("IR_GET_TWIST:: %f %f %f\n", v, theta, omega);
    return twist;
}

// float IR::ComputeV(float frac_v)
// {
//     return frac_v * MAXIMUM_VELOCITY;
// }

// float IR::ComputeOmega(float frac_omega)
// {
//     return frac_omega * MAXIMUM_OMEGA;
// }

// float IR::ThetaFromRawData()
// {
//     return map<float>(rawData, 0, 70, LEFT_THETA_LIMIT, RIGHT_THETA_LIMIT);
// }

// float IR::OmegaFromRawData()
// {
//     return map<float>(rawData, 0, 70, 1.0 * MAXIMUM_OMEGA, -1.0 * MAXIMUM_OMEGA);
// }

void IR::move(float dist, float ang)
{
    v = 0.5 * MAXIMUM_VELOCITY;
    theta = ang;
    omega = 0;

    twist.set_data_from_v_thata_omega(v, theta, omega);
    if (dist != 0)
        HAL_Delay(dist * 1000 / v);
}

void IR::turn(float ang)
{
    v = 0;
    theta = 0; 
    omega = 0.3 * MAXIMUM_VELOCITY;

    twist.set_data_from_v_thata_omega(v, theta, omega);
    if (ang != 0)
        HAL_Delay(ang * 100 / omega);
}