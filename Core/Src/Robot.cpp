#include "Robot.hpp"
#include "Robot_config.h"
#include "usart.h"
#include "dma.h"
#include "tim.h"
#include "Robot.hpp"
#include "Robotlib/filters/ramp.hpp"
#include  <stdio.h>


const float kp[4] = {0.5, 1.2, 0.4, 0.4};
const float ki[4] = {15.0, 20.0, 15.0, 10.5};
const float kd[4] = {0.001, 0.0005, 0.0005, 0.0005};

// const float kp[4] = {1.2, 1.2, 1.2, 1.2};
// const float ki[4] = {16, 16, 16, 16};
// const float kd[4] = {0.002, 0.002, 0.002, 0.002};

const float pid_output_to_speed_factors[4] = {56.52f, 72.22f, 55.89, 52.75f}; 
const float ppr = 1000;

uint8_t prev_button1 = 0x00, prev_button2 = 0x00;

void Robot::init()
{
#ifdef __DEBUG_MODE__
    printf("robot init\n");
#endif

    // 0: M1 -> E3
    // 1: M3 -> E2
    // 2: M4 -> E4
    // 3: M2 -> E1

    base_motors[0] = Motor(&M1P_TIMER, M1D_GPIO_Port, M1P_TIMER_CHANNEL, M1D_Pin);
    base_motors[1] = Motor(&M3P_TIMER, M3D_GPIO_Port, M3P_TIMER_CHANNEL, M3D_Pin);
    base_motors[2] = Motor(&M4P_TIMER, M4D_GPIO_Port, M4P_TIMER_CHANNEL, M4D_Pin);
    base_motors[3] = Motor(&M2P_TIMER, M2D_GPIO_Port, M2P_TIMER_CHANNEL, M2D_Pin);

    base_motor_encoders[0] = Encoder(&ENC3_TIMER, ppr); 
    base_motor_encoders[1] = Encoder(&ENC2_TIMER, ppr); 
    base_motor_encoders[2] = Encoder(&ENC4_TIMER, ppr); 
    base_motor_encoders[3] = Encoder(&ENC1_TIMER, ppr); 

    // const float kus[] = {3.5, 3.5, 1.0, 3.0}; //{0.565, 0.565, 0.55, 0.55};
    // const float tu = 0.05;
    for (int i = 0; i < 4; i++)
    {
        base_motors[i].init();
        base_motor_encoders[i].init();
        base_motor_pid_controllers[i] = PID(1.0, 0.0, 0.0, P_ON_E, DIRECT);

        base_motor_pid_controllers[i].Init();
        base_motor_pid_controllers[i].SetOutputLimits(-pid_output_to_speed_factors[i], pid_output_to_speed_factors[i]);
        base_motor_pid_controllers[i].SetTunings(kp[i], ki[i], kd[i]); // 1.2, 16, 0.01); // 0.2 * kus[i], 0.38 * kus[i] / tu,0.16 * kus[i] * tu);
        base_motor_pid_controllers[i].SetSampleTime(MOTOR_LOOP_TIME);
        base_motor_pid_controllers[i].SetMode(AUTOMATIC);
    }

    joystick.Init();
    deadWheel.init();

    motor_loop = HAL_GetTick();
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

    omniwheel_kinematics.get_motor_omega(base_twist, motor_omegas);
    // printf("omegas: %f %f %f %f\n", motor_omegas[0], motor_omegas[1], motor_omegas[2], motor_omegas[3]);

    // printf("base_motor omega:");
    // for (int i=0; i<4; ++i)
    // {
    //     printf(" %f", motor_omegas[i]);
    // }
    // printf("\n");

    for (int i = 0; i < 4; i++)
    {
        float encoder_omega = base_motor_encoders[i].get_omega();
        base_motor_pid_controllers[i].Input = encoder_omega;

        // printf("%ld ", base_motor_encoders[i].count_aggregate);

        base_motor_pid_controllers[i].Setpoint = motor_omegas[i];

        if (base_motor_pid_controllers[i].Compute())
        {
            base_motors[i].set_speed(base_motor_pid_controllers[i].Output / pid_output_to_speed_factors[i]);
            //  base_motors[i].set_speed(motor_omegas[i]/100);
            base_motor_encoders[i].reset_encoder_count();
        }
    }
    // printf("\n");
}


void Robot::set_state_from_joystick_data(const JoystickData &jdata) 
{
    const uint8_t d_band = 50;

    if ((HAL_GetTick() - joystick.GetLastUpdateTick()) > 100)
    {
#ifdef __DEBUG_MODE__
        // printf("delayed data\n");
#endif
        base_twist = Twist(0, 0, 0);
        return;
    }

    float v = 0, theta = base_twist.get_theta(), omega = 0;

    if (abs(jdata.l_hatx) < d_band && abs(jdata.l_haty) < d_band &&
        jdata.lt < d_band && jdata.rt < d_band)
    {
        base_twist = Twist(0, theta, 0);
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
        speed_factor = 0.2;
    }
    if (jdata.lt > 30)
    {
        omega = map<float>(jdata.lt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
    }
    else if (jdata.rt > 30)
    {
        omega = -map<float>(jdata.rt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
    }

    if ((jdata.button2 & _BV(B_XBOX)) && (jdata.button1 & _BV(B_LB)))
    {
        EMERGENCY_BREAK();
    }

    base_twist = Twist::from_v_theta_omega(v, theta, omega);
    // printf("v th w: %f %f %f\n", v, theta, omega);
}


void Robot::EMERGENCY_BREAK()
{
    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            base_motors[i].set_speed(0.0);
        }
    }
}

void Robot::printJoystickData(const JoystickData& jData)
{
    printf("Received Data: %u %u %u %u %d %d %d %d\n", jData.button1, jData.button2, jData.lt, jData.rt, jData.l_hatx, jData.l_haty, jData.r_hatx, jData.r_haty);
}