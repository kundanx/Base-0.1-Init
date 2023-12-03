#pragma once
#ifndef FUZZY_PID_H_
#define FUZZY_PID_H_

/* Liguistic variables of Fuzzy rule */
#define LB 0  // Low Big
#define LM 1  // Low Medium
#define LS 2  // Low Small
#define ZO 3  // Zero
#define HS 4  // High Small
#define HM 5  // High Medium
#define HB 6  // High Big
#define no_of_fuzzy_sets (7)

class fuzzy_pid {

private:
  float outMax, outMin, error, d_error, error_l, error_ll, del_out, output;
  float kp, ki, kd, A, B, C, maxValue, maxIndex[7][7];
  float e_sets[no_of_fuzzy_sets], de_sets[no_of_fuzzy_sets];
  int maxRow, maxColumn;

public:
  fuzzy_pid();
  void set_parameter(float, float, float, float, float);
  float trimf(float, float, float, float);
  float invTrimf(float, float, float, float);
  void calculate_del_output(float, float, float);
  float calculate_del_tuningGain(int[no_of_fuzzy_sets][no_of_fuzzy_sets],
                                 float[no_of_fuzzy_sets * 3]);
  float map(float, float, float);
  float clamp(float, float, float);
  float compute_fuzzy_selfTuning_PID(float, float);
  void get_parameter(float[3]);


  // API that's closer to PID
  void setTunings(double, double, double);
  void SetOutputLimits(double, double);
  bool Compute();

};

#endif //! FUZZY_PID_H_
