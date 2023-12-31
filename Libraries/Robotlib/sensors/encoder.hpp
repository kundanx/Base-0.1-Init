#ifndef ROBOTLIB_ENCODER_HPP_
#define ROBOTLIB_ENCODER_HPP_

#include "stm32f4xx.h"

class Encoder
{
public:
  TIM_HandleTypeDef *henc;
  uint16_t ppr = 1;
  float omega = 0;
  int32_t count_aggregate = 0, prevCount = 0;
  uint32_t last_reset_time = 0;

  Encoder() {};
  Encoder(TIM_HandleTypeDef *_henc, uint16_t _ppr)
      : henc(_henc), ppr(_ppr) {}

  float get_omega(void);
  void init(void);
  int16_t get_count(void);
  int64_t get_count_aggregate(void);
  void reset_encoder_count(void);
};

#endif // ROBOTLIB_ENCODER_HPP_
