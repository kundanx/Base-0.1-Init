#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.2.1

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

class PID
{

public:
// Constants used in some of the functions below
#define AUTOMATIC 1 // For Auto Tunnig
#define MANUAL 0    // For Manual Tunning
#define P_ON_M 0    // Proportional On Measurement
#define P_ON_E 1    // Proportional On Error

/* Direct: +ve Pid Ouput leads +ve change in System Output
 * Reversse: +ve Pid Output leads -ve change in System Output
 */
#define DIRECT 0
#define REVERSE 1

  PID();

  PID(double _kp, double _ki, double _kd, int _POn, int _ControllerDirection);

  // commonly used functions
  // **************************************************************************
  // PID(double *, double *,
  //     double *, // * constructor.  links the PID to the Input, Output, and
  //     double, double, double, int,
  //     int); //   Setpoint.  Initial tuning parameters are also set here.
  //   (overload for specifying proportional mode)

  // PID(double *, double *,
  //     double *, // * constructor.  links the PID to the Input, Output, and
  //     double, double, double,
  //     int); //   Setpoint.  Initial tuning parameters are also set here

  void SetMode(int Mode); // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute(); // * performs the PID calculation.  it should be
                  //   called every time loop() cycles. ON/OFF and
                  //   calculation frequency can be set using SetMode
                  //   SetSampleTime respectively

  void SetOutputLimits(
      double,
      double); // * clamps the output to a specific range. 0-255 by default, but
               //   it's likely the user will want to change this depending on
               //   the application

  // available but not commonly used functions
  // ********************************************************
  void SetTunings(
      double, double,             // * While most users will set the tunings once in the
      double);                    //   constructor, this function gives the user the option
                                  //   of changing tunings during runtime for Adaptive control
  void SetTunings(double, double, // * overload for specifying proportional mode
                  double, int);

  void SetControllerDirection(
      int); // * Sets the Direction, or "Action" of the controller. DIRECT
            //   means the output will increase when error is positive. REVERSE
            //   means the opposite.  it's very unlikely that this will be
            //   needed once it is set in the constructor.
  void
  SetSampleTime(int); // * sets the frequency, in Milliseconds, with which
                      //   the PID calculation is performed.  default is 100

  // Display functions
  // ****************************************************************
  double GetKp();     // These functions query the pid for interal values.
  double GetKi();     //  they were created mainly for the pid front-end,
  double GetKd();     // where it's important to know what is actually
  int GetMode();      //  inside the PID.
  int GetDirection(); //
  double outputSum = 0;
  void Init();

  double Input = 0, Output = 0, Setpoint = 0;

private:
  void Initialize();

  double dispKp; // * we'll hold on to the tuning parameters in user-entered
  double dispKi; //   format for display purposes
  double dispKd; //

  double kp; // * (P)roportional Tuning Parameter
  double ki; // * (I)ntegral Tuning Parameter
  double kd; // * (D)erivative Tuning Parameter

  int controllerDirection;
  int pOn; // Proportional on Error(POE) + Proportional on Measurement(POM)
           // If pOn = P_ON_M i.e. 0, POE is false and POM is true.
           // If pOn = P_ON_E i.e. 1, POE is true and POM is false.
           // If pOn is neither 1 nor 0, both of them are false.

  double *myInput;  // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput; //   This creates a hard link between the variables and the
  double *mySetpoint; //   PID, freeing the user from having to constantly tell us
                   //   what these values are.  with pointers we'll just know.

  unsigned long lastTime;
  double lastInput = 0;

  unsigned long SampleTime;
  double outMin, outMax;
  bool inAuto, pOnE;
};
#endif
