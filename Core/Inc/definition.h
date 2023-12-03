#ifndef DEFINITION_H__
#define DEFINITION_H__


#define __DEBUG__       1
#define __CRC__         0
#define __FUZZY__       0

/********************************************************************************
 * Defintion for implementation
 *******************************************************************************/

#if __DEBUG__ == 1
#define __DEBUG_MODE__
#endif

#if __CRC__ == 1
#define __IMPLEMENT_CRC__
#else
#define __IMPLEMENT_CHECKSUM__
#endif

#if __FUZZY__ == 1
#define  _FUZZY_PID
#else
#define _CLASSICAL_PID
#endif



#define MOTOR_LOOP_TIME (uint32_t)10
#define ROBOT_LOOP_TIME (uint32_t)10


/*********************************************************************************
 * Robot Dimensions
*********************************************************************************/
#define BASE_DIAMETER (0.73)
#define WHEEL_DIAMETER (0.13)
#define MAXIMUM_VELOCITY (1.0)
#define MAXIMUM_OMEGA (2.0)


/********************************************************************************
 * Joystick control definitions 
*********************************************************************************/
#define B_X (7)
#define B_Y (6)
#define B_A (5)
#define B_B (4)
#define B_UP (3)
#define B_DOWN (2)
#define B_LB (1)
#define B_RB (0)

#define B_START (7)
#define B_BACK (6)
#define B_XBOX (5)
#define B_LEFT (4)
#define B_RIGHT (3)
#define B_L3 (2)
#define B_R3 (1)

#define _BV(x) (1 << x)



#endif // DEFINITION_H__