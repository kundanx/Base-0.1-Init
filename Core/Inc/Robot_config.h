#ifndef ROBOT_CONFIG_H__
#define ROBOT_CONFIG_H__


#define JOYSTICK_UART           (huart2)

#define M1P_TIMER               (htim8)
#define M2P_TIMER               (htim5)
#define M3P_TIMER               (htim9)
#define M4P_TIMER               (htim9)

#define M1P_TIMER_CHANNEL       (TIM_CHANNEL_1) // N CHANNEL
#define M2P_TIMER_CHANNEL       (TIM_CHANNEL_4)
#define M3P_TIMER_CHANNEL       (TIM_CHANNEL_1)
#define M4P_TIMER_CHANNEL       (TIM_CHANNEL_2)

#define ENC1_TIMER              (htim1)
#define ENC2_TIMER              (htim3)
#define ENC3_TIMER              (htim4)
#define ENC4_TIMER              (htim2)

#endif // ROBOT_CONFIG_H__