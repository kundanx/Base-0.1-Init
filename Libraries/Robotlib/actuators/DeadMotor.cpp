#include "DeadMotor.hpp"
#include <stdio.h>
#include "Robotlib/maths/math.hpp"

const float kp[4] = {0.5, 1.2, 0.4, 0.4};
const float ki[4] = {15.0, 20.0, 15.0, 10.5};
const float kd[4] = {0.001, 0.0005, 0.0005, 0.0005};
const float pid_output_to_speed_factors[4] = {56.52f, 72.22f, 55.89, 52.75f};
const float cpr = 1000;

const float pos_kp = 5.0f;
const float pos_ki = 0.0f;
const float pos_kd = 0.00f;
const float pos_pid_limit = 1.0f; // full speed when greater or equal to 5m

const float theta_kp = 5.0;
const float theta_ki = 0.0;
const float theta_kd = 0.000;
const float theta_pid_limit = PI/6.0f;

void DeadMotor::init()
{
    // 0: M1 -> E3
    // 1: M3 -> E2
    // 2: M4 -> E4
    // 3: M2 -> E1

    base_motors[0] = Motor(&M1P_TIMER, M1D_GPIO_Port, M1P_TIMER_CHANNEL, M1D_Pin);
    base_motors[1] = Motor(&M3P_TIMER, M3D_GPIO_Port, M3P_TIMER_CHANNEL, M3D_Pin);
    base_motors[2] = Motor(&M4P_TIMER, M4D_GPIO_Port, M4P_TIMER_CHANNEL, M4D_Pin);
    base_motors[3] = Motor(&M2P_TIMER, M2D_GPIO_Port, M2P_TIMER_CHANNEL, M2D_Pin);

    base_motor_encoders[0] = Encoder(&ENC3_TIMER, cpr);
    base_motor_encoders[1] = Encoder(&ENC2_TIMER, cpr);
    base_motor_encoders[2] = Encoder(&ENC4_TIMER, cpr);
    base_motor_encoders[3] = Encoder(&ENC1_TIMER, cpr);

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

    x_pid = PID(1.0, 0.0, 0.0, P_ON_E, DIRECT);
    x_pid.Init();
    x_pid.SetOutputLimits(-pos_pid_limit, pos_pid_limit);
    x_pid.SetTunings(pos_kp, pos_ki, pos_kd);
    x_pid.SetSampleTime(MOTOR_LOOP_TIME);
    x_pid.SetMode(AUTOMATIC);

    y_pid = PID(1.0, 0.0, 0.0, P_ON_E, DIRECT);
    y_pid.Init();
    y_pid.SetOutputLimits(-pos_pid_limit, pos_pid_limit);
    y_pid.SetTunings(pos_kp, pos_ki, pos_kd);
    y_pid.SetSampleTime(MOTOR_LOOP_TIME);
    y_pid.SetMode(AUTOMATIC);

    theta_pid = PID(1.0, 0.0, 0.0, P_ON_E, DIRECT);
    theta_pid.Init();
    theta_pid.SetOutputLimits(-theta_pid_limit, theta_pid_limit);
    theta_pid.SetTunings(theta_kp, theta_ki, theta_kd);
    theta_pid.SetSampleTime(MOTOR_LOOP_TIME);
    theta_pid.SetMode(AUTOMATIC);

    deadWheel.init();

    motor_loop = HAL_GetTick();
}

void DeadMotor::run()
{
    if ((HAL_GetTick() - motor_loop) >= MOTOR_LOOP_TIME)
    {

        omniwheel_kinematics.get_motor_omega(base_twist, motor_omegas);

        for (int i = 0; i < 4; i++)
        {
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
        }
        // printf("\n");

        motor_loop = HAL_GetTick();
    }
}

float xVel = 0, yVel = 0, newOmega = 0;
void DeadMotor::maintainCoordinates()
{
    
    theta_pid.Input = round4(odom.theta);
    theta_pid.Setpoint = odom_setpoint.theta;
    if (theta_pid.Compute())
    {
        newOmega = theta_pid.Output / pos_pid_limit * MAXIMUM_OMEGA;
    }

    x_pid.Input = round3(odom.x);
    x_pid.Setpoint = odom_setpoint.x;
    if (x_pid.Compute())
    {
        xVel = x_pid.Output / pos_pid_limit * MAXIMUM_VELOCITY;
    }

    y_pid.Input = round3(odom.y);
    y_pid.Setpoint = odom_setpoint.y;
    if (y_pid.Compute())
    {
        yVel = y_pid.Output / pos_pid_limit * MAXIMUM_VELOCITY;
    }

    base_twist = Twist(xVel, yVel, newOmega);

    run();
}

void DeadMotor::changeCoordinates(float x, float y, float theta)
{
    odom_setpoint.x = x;
    odom_setpoint.y = y;
    odom_setpoint.theta = theta;
}

void DeadMotor::changeTheta(float newTheta)
{
    odom_setpoint.theta = newTheta;
    if (odom_setpoint.theta > M_PI)
    {
        odom_setpoint.theta -= 2 * M_PI;
    }
    else if (odom_setpoint.theta < (-M_PI))
    {
        odom_setpoint.theta += 2 * M_PI;
    }

    odom_setpoint.theta = round3(odom_setpoint.theta);
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