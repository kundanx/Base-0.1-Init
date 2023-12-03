#include "DeadMotor.hpp"
#include <stdio.h>

#ifdef _CLASSICAL_PID
// const float kp[4] = {1.0, 1.0, 1.0, 1.0};
// const float ki[4] = {15, 15, 15, 15};
// const float kd[4] = {0.0, 0.01, 0.01, 0.01};

const float kp[4] = {2.0f, 2.5f, 2.5f, 2.0f};
const float ki[4] = {15.0f, 15.0f, 15.0f, 15.0f};
const float kd[4] = {0.01f, 0.0f, 0.0, 0.01f};
#endif

#ifdef _FUZZY_PID
const float kp[4] = {2, 2, 2, 2};
const float ki[4] = {20, 20, 30, 20};
const float kd[4] = {0.009, 0.009, 0.009, 0.008};
#endif

const float pid_output_to_speed_factors[4] = {56.52f, 72.22f, 55.89, 52.75f};
const float cpr = 1000;

void DeadMotor::init()
{
    // 0: M1 -> E3
    // 1: M3 -> E2
    // 2: M4 -> E4
    // 3: M2 -> E1

    base_motors[0] = Motor(&M1P_TIMER, M1D_GPIO_Port, M1P_TIMER_CHANNEL, M1D_Pin, true);
    base_motors[1] = Motor(&M3P_TIMER, M3D_GPIO_Port, M3P_TIMER_CHANNEL, M3D_Pin);
    base_motors[2] = Motor(&M4P_TIMER, M4D_GPIO_Port, M4P_TIMER_CHANNEL, M4D_Pin);
    base_motors[3] = Motor(&M2P_TIMER, M2D_GPIO_Port, M2P_TIMER_CHANNEL, M2D_Pin);

    base_motor_encoders[0] = Encoder(&ENC3_TIMER, cpr);
    base_motor_encoders[1] = Encoder(&ENC2_TIMER, cpr);
    base_motor_encoders[2] = Encoder(&ENC4_TIMER, cpr);
    base_motor_encoders[3] = Encoder(&ENC1_TIMER, cpr);

#ifdef _CLASSICAL_PID
    for (int i = 0; i < 4; i++)
    {
        base_motors[i].init();
        base_motor_encoders[i].init();
        base_motor_pid_controllers[i] = PID(1.0, 0.0, 0.0, P_ON_E, DIRECT);

        base_motor_pid_controllers[i].Init();
        base_motor_pid_controllers[i].SetOutputLimits(-pid_output_to_speed_factors[i], pid_output_to_speed_factors[i]);
        base_motor_pid_controllers[i].SetTunings(kp[i], ki[i], kd[i]);
        base_motor_pid_controllers[i].SetSampleTime(MOTOR_LOOP_TIME);
        base_motor_pid_controllers[i].SetMode(AUTOMATIC);
    }
#endif

#ifdef _FUZZY_PID
    for (int i = 0; i < 4; i++)
    {
        base_motor_fuzzy_pid[i] = fuzzy_pid();
        base_motors[i].init();
        base_motor_encoders[i].init();
        base_motor_fuzzy_pid[i].set_parameter(kp[i], ki[i], kd[i], -pid_output_to_speed_factors[i], pid_output_to_speed_factors[i]);
    }
#endif

    motor_loop = HAL_GetTick();
}

void DeadMotor::run()
{
    if ((HAL_GetTick() - motor_loop) >= MOTOR_LOOP_TIME)
    {

        omniwheel_kinematics.get_motor_omega(base_twist, motor_omegas);

        for (int i = 0; i < 4; i++)
        {
#ifdef _CLASSICAL_PID
            base_motor_pid_controllers[i].Input = base_motor_encoders[i].get_omega();

            // printf("%f ", base_motor_encoders[i].omega);
            // printf("%ld   ", base_motor_encoders[i].count_aggregate);

            base_motor_pid_controllers[i].Setpoint = motor_omegas[i];

            if (base_motor_pid_controllers[i].Compute())
            {
                base_motors[i].set_speed(base_motor_pid_controllers[i].Output / pid_output_to_speed_factors[i]);
                //  base_motors[i].set_speed(motor_omegas[i]/100);
                base_motor_encoders[i].reset_encoder_count();
            }

#endif

#ifdef _FUZZY_PID
            base_motor_pid_inputs[i] = base_motor_encoders[i].get_omega();
            base_motor_pid_setpoints[i] = motor_omegas[i];
            base_motor_pid_outputs[i] = base_motor_fuzzy_pid[i].compute_fuzzy_selfTuning_PID(base_motor_pid_setpoints[i],
                                                                                             base_motor_pid_inputs[i]);
            base_motors[i].set_speed(base_motor_pid_outputs[i] / pid_output_to_speed_factors[i]);
            base_motor_encoders[i].reset_encoder_count();
#endif
        }
        // printf("\n");

        motor_loop = HAL_GetTick();
    }
}

void DeadMotor::move(float velocity, float theta, float omega)
{
    base_twist = Twist::from_v_theta_omega(velocity, theta, omega);
}

void DeadMotor::linearMove(float disp, float theta, float velocity)
{
    base_twist = Twist::from_v_theta_omega(velocity, theta, 0);
    uint32_t time = 1000 * disp / velocity;

    uint32_t refTime = HAL_GetTick();
    motor_loop  = refTime;

        // printf("running\n");
    while ((motor_loop - refTime) < time)
    {
        // printf("run\n");
        run();
    }
    stop();
}

void DeadMotor::angularMove(float disp, float omega)
{
    base_twist = Twist(0, 0, omega);
    uint32_t time = 1000 * disp / omega;

    uint32_t refTime = HAL_GetTick();
    motor_loop  = refTime;

    while ((motor_loop - refTime) < time)
    {
        run();
    }
    stop();
}

void DeadMotor::stop()
{
    base_twist = Twist(0, 0, 0);
}

void DeadMotor::hardBreak()
{
    for (int i = 0; i < 4; i++)
    {
        base_motors[i].set_speed(0.0);
    }
}