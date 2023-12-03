#ifndef __IR_SENSOR_LIB__
#define __IR_SENSOR_LIB__

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "control_struct.h"
#include "Robotlib/maths/math.hpp"
#include "Robotlib/controllers/pid.hpp"

#define MEM_CENTER  0
#define MEM_LEFT    1
#define MEM_RIGHT   2

class IR
{
public:
    IR(UART_HandleTypeDef* _huart);
    IR() {};
    ~IR() {};

    void Init();
    void RxCallback();
    void UpdateJP();
    bool Follow();
    void GetData();
    Twist GetTwist();

    UART_HandleTypeDef* huart;
    uint32_t lastUpdateTick;
    PID pid;

    uint32_t CJPCount = 0;
    uint32_t LJPCount = 0;
    uint32_t RJPCount = 0;
    uint8_t rawData;
    bool isCJP = false;
    bool isLJP = false;
    bool isRJP = false;

    void move(float dist, float theta);
    void turn(float ang);


private:

    // float ComputeV(float frac_v);
    // float ComputeOmega(float frac_omega);
    // float ThetaFromRawData();
    // float OmegaFromRawData();

    uint8_t buffer = 100; // Avoiding 0

    Twist twist{0, 0, 0};
    float v = 0, theta = 0, omega = 0;
    uint8_t mem = 0;
    
    const float LEFT_THETA_LIMIT = degToRad<float>(90.0 + 12.5);
    const float RIGHT_THETA_LIMIT = degToRad<float>(90.0 - 12.5);
};

#endif