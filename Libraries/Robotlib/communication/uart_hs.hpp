/******************************************************************************
 *                             Uart Handshake 
 * 
 *  - Created by Sagar @Robotics Club, Pulchowk Campus
*******************************************************************************/

#ifndef __UART_HS
#define __UART_HS


/********************************************************************************
 * Includes
********************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "definition.h"

/* Include extra crc library if implemented */
#ifdef __IMPLEMENT_CRC__
#include "Robotlib/crypto/crc.hpp"
#endif


/******************************************************************************
 * Data Format definitions
*******************************************************************************/
#define START_BYTE 0xA5
#define BUFFER_SIZE 10
#define PACKET_SIZE 10
#define DATA_SIZE 8

#ifdef __IMPLEMENT_CRC__
#define CRC_POLYNOMIAL 7
#endif


/******************************************************************************
 * Joystick data structure
 * 
 * There are ten bytes in a packet of control data.
 * First byte is start byte.
 * Last byte is error detection byte.
 * Between eigth bytes include followings:
 * 
 * Button byte-1: X Y A B Up Down lb RB
 * Button byte-2: Start Back XBox Left right Left-hat-press Right-hat-press 0
 * LT
 * RT
 * Left-hat-x
 * Left-hat-y
 * Right-hat-x
 * Right-hat-y
*******************************************************************************/
struct JoystickData
{
  uint8_t button1 = 0;
  uint8_t button2 = 0;
  uint8_t lt = 0;
  uint8_t rt = 0;
  int8_t l_hatx = 0;
  int8_t l_haty = 0;
  int8_t r_hatx = 0;
  int8_t r_haty = 0;
};


/******************************************************************************
 * UartHs Class
*******************************************************************************/
class UartHS
{
public:
  UartHS() {};
  UartHS(UART_HandleTypeDef *_huart);
  ~UartHS() {};

  void Init();
  void RxCallback();
  void GetData(JoystickData &jdata_);

#ifdef __DEBUG_MODE__
  void ShowData();
  void ShowPacket();
  void ShowStatus();
#endif

  UART_HandleTypeDef *huart;
  uint32_t GetLastUpdateTick();

private:
  bool VerifyPacket();
  void UpdateData();

  void CheckError();
  void IndicateError();

  JoystickData data;

  uint8_t buffer[BUFFER_SIZE];
  uint8_t handshakeByte = START_BYTE;
  uint32_t last_update_tick;

  uint8_t status;
  GPIO_TypeDef *RX_CALLBACK_INDICATOR_Port = GREEN_LED_GPIO_Port;
  uint16_t RX_CALLBACK_INDICATOR_Pin = GREEN_LED_Pin;
  GPIO_TypeDef *ERROR_INDICATOR_Port = RED_LED_GPIO_Port;
  uint16_t ERROR_INDICATOR_Pin = RED_LED_Pin;

#ifdef __IMPLEMENT_CRC__
  CRC_Hash crc{CRC_POLYNOMIAL};
#endif
};

#endif // __UART_HS