/*****************************************************************************
 *                          UART using Handshake
 * 
 *  Request data by sending byte (start byte)
 *  To transmit, interrupt mode is used.
 *  To receive, DMA mode is used.
 *  
 * Advantages:
 * - No buffer full and overide issue.
 * - Start byte get received at the start position.
 * - Less overhead of error checking. 
*****************************************************************************/

#include "uart_hs.hpp"
#include "dma.h"
#include "usart.h"
#include <stdio.h>

/*****************************************************************************
 * Initialize uart handle using paramentric contructor
*****************************************************************************/
UartHS::UartHS(UART_HandleTypeDef *_huart)
    : huart(_huart) {}

/*****************************************************************************
 * Start communication
 * 
 * Transmit one byte using interrupt for request of data
 * Receive one packet using DMA
*****************************************************************************/
void UartHS::Init()
{
    last_update_tick = HAL_GetTick();

    HAL_UART_Transmit_IT(huart, &handshakeByte, 1);

    status = HAL_UART_Receive_DMA(huart, buffer, 10);
}

/******************************************************************************
 *  Callback response on complete packet receive
 *****************************************************************************/
void UartHS::RxCallback()
{
    static uint32_t prevTick = 0;
    uint32_t curTick = HAL_GetTick();

    /* Toggle LED to indicate data recive */
    if ((curTick - prevTick) > 50)
    {
        HAL_GPIO_TogglePin(RX_CALLBACK_INDICATOR_Port, RX_CALLBACK_INDICATOR_Pin);
        prevTick = curTick;
    }

    /* Verify Packet before transferrinf buffer data to control data*/
    if (VerifyPacket())
    {
        /* Update contro data */
        UpdateData();
    }
    else
    {
        /* Response to error */
        IndicateError();
    }

    /* Request again */
    HAL_UART_Transmit_IT(huart, &handshakeByte, 1);

    /* Receive again */
    status = HAL_UART_Receive_DMA(huart, buffer, 10);
}

/*******************************************************************************
 * Funtion to verify packet
******************************************************************************/
bool UartHS::VerifyPacket()
{
    if (buffer[0] == START_BYTE)
    {
#ifdef __IMPLEMENT_CHECKSUM__
        /* Calculate checksum and compare */
        uint8_t checksum = buffer[0];

        for (int i = 1; i < (PACKET_SIZE - 1); i++)
        {
            checksum += buffer[i];
        }

        if (checksum == buffer[PACKET_SIZE - 1])
            return true;
#endif

#ifdef __IMPLEMENT_CRC__
    /* We use table methode to generate remainder */
        uint8_t hash = crc.get_Hash(buffer, BUFFER_SIZE - 1);
    /* Check remainder */
        if (hash == buffer[PACKET_SIZE - 1]);
        return true;
#endif
    }

    return false;
}

/*****************************************************************************
 * Function to update data from buffer
******************************************************************************/
void UartHS::UpdateData()
{
    data.button1 = buffer[1];
    data.button2 = buffer[2];
    data.lt = buffer[3];
    data.rt = buffer[4];
    data.l_hatx = (int8_t)buffer[5];
    data.l_haty = (int8_t)buffer[6];
    data.r_hatx = (int8_t)buffer[7];
    data.r_haty = (int8_t)buffer[8];

    last_update_tick = HAL_GetTick();
}

/******************************************************************************
 * Transfer data to control variable of Robot
 *****************************************************************************/
void UartHS::GetData(JoystickData &jdata_)
{
    CheckError();

    jdata_ = data;
}

/****************************************************************************
 * Check error
 * 
 * Error can be due to error in data packet, no callback or slow receove.
***************************************************************************/
void UartHS::CheckError()
{
    if ((status != HAL_OK) || ((HAL_GetTick() - last_update_tick) > 1000))
    {
        IndicateError();
        status = HAL_UART_Receive_DMA(huart, buffer, 10);
    }
}

/*****************************************************************************
 * Function to indicate error
 *
 * Toggle LED
*****************************************************************************/
void UartHS::IndicateError()
{
    static uint32_t prevTick = 0;
    uint32_t curTick = HAL_GetTick();

    if ((curTick - prevTick) > 50)
    {
        HAL_GPIO_TogglePin(ERROR_INDICATOR_Port, ERROR_INDICATOR_Pin);
        prevTick = curTick;
    }
}

/*******************************************************************************
 * Provide last data update tick
 *******************************************************************************/
uint32_t UartHS::GetLastUpdateTick()
{
    return last_update_tick;
}

#ifdef __DEBUG_MODE__
/******************************************************************************
 * Display Packet received
******************************************************************************/
void UartHS::ShowPacket()
{
    CheckError();

    printf("Joystick::PACKET:");
    for (int i = 0; i < PACKET_SIZE; ++i)
    {
        printf(" %u", buffer[i]);
    }
    printf("\n");
}

/*****************************************************************************
 * Show active data
******************************************************************************/
void UartHS::ShowData()
{
    CheckError();

    printf("Joystick::data: %u %u %u %u %d %d %d %d\n",
           data.button1,
           data.button2,
           data.lt,
           data.rt,
           data.l_hatx,
           data.l_haty,
           data.r_hatx,
           data.r_haty);
}

/*******************************************************************************
 * Show Status
*******************************************************************************/
void UartHS::ShowStatus()
{
    printf("Joystick::status: %d  \n", status);
}
#endif // __DEBUG_MODE__
