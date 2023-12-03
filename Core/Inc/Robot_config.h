#ifndef ROBOT_CONFIG_H__
#define ROBOT_CONFIG_H__

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"

#define IR_JPC_PORT             GPIOB                       
#define IR_JPLL_PORT            GPIOE               
#define IR_JPLR_PORT            GPIOD         
#define IR_JPRL_PORT            GPIOC            
#define IR_JPRR_PORT            GPIOC    

#define IR_JPC_PIN              GPIO_PIN_12                       
#define IR_JPLL_PIN             GPIO_PIN_12          
#define IR_JPLR_PIN             GPIO_PIN_10      
#define IR_JPRL_PIN             GPIO_PIN_6      
#define IR_JPRR_PIN             GPIO_PIN_7

#define JOYSTICK_UART           huart2
#define IR_UART                 huart4

#define M1P_TIMER               htim8
#define M2P_TIMER               htim5
#define M3P_TIMER               htim9
#define M4P_TIMER               htim9

#define M1P_TIMER_CHANNEL       TIM_CHANNEL_1 // N CHANNEL
#define M2P_TIMER_CHANNEL       TIM_CHANNEL_4
#define M3P_TIMER_CHANNEL       TIM_CHANNEL_1
#define M4P_TIMER_CHANNEL       TIM_CHANNEL_2

#define ENC1_TIMER              htim1
#define ENC2_TIMER              htim3
#define ENC3_TIMER              htim4
#define ENC4_TIMER              htim2

#endif // ROBOT_CONFIG_H__