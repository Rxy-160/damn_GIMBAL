/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "MCP2515.h"
#include "gpio.h"

/* SPI related variables */
#define SPI_TIMEOUT             10

void WHW_MCP2515_CS_HIGH(SPI_HandleTypeDef *hspi)
{
//    if (hspi == &hspi1)
//        HAL_GPIO_WritePin(SPI_CAN_1_CS_GPIO_Port, SPI_CAN_1_CS_Pin, GPIO_PIN_SET);
    if (hspi == &hspi2)
        HAL_GPIO_WritePin(SPI_CAN_2_CS_GPIO_Port, SPI_CAN_2_CS_Pin, GPIO_PIN_SET);
}

void WHW_MCP2515_CS_LOW(SPI_HandleTypeDef *hspi)
{
//    if (hspi == &hspi1)
//        HAL_GPIO_WritePin(SPI_CAN_1_CS_GPIO_Port, SPI_CAN_1_CS_Pin, GPIO_PIN_RESET);
    if (hspi == &hspi2)
        HAL_GPIO_WritePin(SPI_CAN_2_CS_GPIO_Port, SPI_CAN_2_CS_Pin, GPIO_PIN_RESET);
}

#define MCP2515_CS_HIGH(x)   WHW_MCP2515_CS_HIGH(x);
#define MCP2515_CS_LOW(x)    WHW_MCP2515_CS_LOW(x);

/* Prototypes */
static void SPI_Tx(SPI_HandleTypeDef *hspi, uint8_t data);
static void SPI_TxBuffer(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(SPI_HandleTypeDef *hspi);
static void SPI_RxBuffer(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint8_t length);

/* initialize MCP2515 */
bool MCP2515_Initialize(SPI_HandleTypeDef *hspi)
{
  MCP2515_CS_HIGH(hspi)
  
  uint8_t loop = 10;
  
  do {
    /* check SPI Ready */
    if(HAL_SPI_GetState(hspi) == HAL_SPI_STATE_READY)
      return true;
    
    loop--;
  } while(loop > 0); 
      
  return false;
}

/* change mode as configuration mode */
bool MCP2515_SetConfigMode(SPI_HandleTypeDef *hspi)
{
  /* configure CANCTRL Register */
  MCP2515_WriteByte(hspi, MCP2515_CANCTRL, 0x80);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP2515_ReadByte(hspi, MCP2515_CANSTAT) & 0xE0) == 0x80)
      return 1;
    
    loop--;
  } while(loop > 0); 
  
  return 0;
}

/* change mode as normal mode */
bool MCP2515_SetNormalMode(SPI_HandleTypeDef *hspi)
{
  /* configure CANCTRL Register */
  MCP2515_WriteByte(hspi, MCP2515_CANCTRL, 0x00);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP2515_ReadByte(hspi, MCP2515_CANSTAT) & 0xE0) == 0x00)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* Entering sleep mode */
bool MCP2515_SetSleepMode(SPI_HandleTypeDef *hspi)
{
  /* configure CANCTRL Register */
  MCP2515_WriteByte(hspi, MCP2515_CANCTRL, 0x20);
  
  uint8_t loop = 10;
  
  do {    
    /* confirm mode configuration */
    if((MCP2515_ReadByte(hspi, MCP2515_CANSTAT) & 0xE0) == 0x20)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* MCP2515 SPI-Reset */
void MCP2515_Reset(SPI_HandleTypeDef *hspi)
{    
  MCP2515_CS_LOW(hspi)
      
  SPI_Tx(hspi, MCP2515_RESET);
      
  MCP2515_CS_HIGH(hspi)
}

/* read single byte */
uint8_t MCP2515_ReadByte (SPI_HandleTypeDef *hspi, uint8_t address)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, MCP2515_READ);
  SPI_Tx(hspi, address);
  retVal = SPI_Rx(hspi);
      
  MCP2515_CS_HIGH(hspi)
  
  return retVal;
}

/* read buffer */
void MCP2515_ReadRxSequence(SPI_HandleTypeDef *hspi, uint8_t instruction, uint8_t *data, uint8_t length)
{
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, instruction);
  SPI_RxBuffer(hspi, data, length);
    
  MCP2515_CS_HIGH(hspi)
}

/* write single byte */
void MCP2515_WriteByte(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t data)
{    
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, MCP2515_WRITE);
  SPI_Tx(hspi, address);
  SPI_Tx(hspi, data);
    
  MCP2515_CS_HIGH(hspi)
}

/* write buffer */
void MCP2515_WriteByteSequence(SPI_HandleTypeDef *hspi, uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{    
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, MCP2515_WRITE);
  SPI_Tx(hspi, startAddress);
  SPI_TxBuffer(hspi, data, (endAddress - startAddress + 1));
  
  MCP2515_CS_HIGH(hspi)
}

/* write to TxBuffer */
void MCP2515_LoadTxSequence(SPI_HandleTypeDef *hspi, uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{    
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, instruction);
  SPI_TxBuffer(hspi, idReg, 4);
  SPI_Tx(hspi, dlc);
  SPI_TxBuffer(hspi, data, dlc);
       
  MCP2515_CS_HIGH(hspi)
}

/* write to TxBuffer(1 byte) */
void MCP2515_LoadTxBuffer(SPI_HandleTypeDef *hspi, uint8_t instruction, uint8_t data)
{
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, instruction);
  SPI_Tx(hspi, data);
        
  MCP2515_CS_HIGH(hspi)
}

/* request to send */
void MCP2515_RequestToSend(SPI_HandleTypeDef *hspi, uint8_t instruction)
{
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, instruction);
      
  MCP2515_CS_HIGH(hspi)
}

/* read status */
uint8_t MCP2515_ReadStatus(SPI_HandleTypeDef *hspi)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, MCP2515_READ_STATUS);
  retVal = SPI_Rx(hspi);
        
  MCP2515_CS_HIGH(hspi)
  
  return retVal;
}

/* read RX STATUS register */
uint8_t MCP2515_GetRxStatus(SPI_HandleTypeDef *hspi)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, MCP2515_RX_STATUS);
  retVal = SPI_Rx(hspi);
        
  MCP2515_CS_HIGH(hspi)
  
  return retVal;
}

/* Use when changing register value */
void MCP2515_BitModify(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t mask, uint8_t data)
{    
  MCP2515_CS_LOW(hspi)
  
  SPI_Tx(hspi, MCP2515_BIT_MOD);
  SPI_Tx(hspi, address);
  SPI_Tx(hspi, mask);
  SPI_Tx(hspi, data);
        
  MCP2515_CS_HIGH(hspi)
}

/* SPI Tx wrapper function  */
static void SPI_Tx(SPI_HandleTypeDef *hspi, uint8_t data)
{
  HAL_SPI_Transmit(hspi, &data, 1, SPI_TIMEOUT);
}

/* SPI Tx wrapper function */
static void SPI_TxBuffer(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(hspi, buffer, length, SPI_TIMEOUT);
}

/* SPI Rx wrapper function */
static uint8_t SPI_Rx(SPI_HandleTypeDef *hspi)
{
  uint8_t retVal;
  HAL_SPI_Receive(hspi, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

/* SPI Rx wrapper function */
static void SPI_RxBuffer(SPI_HandleTypeDef *hspi, uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Receive(hspi, buffer, length, SPI_TIMEOUT);
}
