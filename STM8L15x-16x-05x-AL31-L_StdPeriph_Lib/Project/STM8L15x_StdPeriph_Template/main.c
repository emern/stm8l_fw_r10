/**
  ******************************************************************************
  * @file    GPIO/GPIO_Toggle/main.c
  * @author  MCD Application Team
  * @version V1.5.2
  * @date    30-September-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include <string.h>


// #define PUTCHAR_PROTOTYPE char putchar (char c)
// #define GETCHAR_PROTOTYPE char getchar (void)
/** @addtogroup GPIO_Toggle
  * @{
  */

#define LED_GPIO_PORT  GPIOA
#define LED_GPIO_PINS  GPIO_Pin_4
#define I2C_SPEED 100000
#define ARR_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define TEN_SEC_BITMASK 0b01110000
#define SECONDS_BITMASK 0b00001111
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay (uint16_t nCount);
char putchar (char c);
char getchar (void);
void tiny_print (char* str, int len);
void tiny_scan (char* out, int len);
void i2c_write(uint8_t data);
uint8_t read_rtc(void);
uint8_t decode_rtc(uint8_t val);
void print_secs(uint8_t secs);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
	char ans;
  uint8_t i2c_resp = 0;
  uint8_t time_dec = 0;
  uint8_t time_dec_last = 0;
  uint8_t addr = 208;
  uint8_t own = 15;
  char greeting[] = "HELLO\r\n";
  char response[5];

  /*High speed internal clock prescaler: 1*/
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_2);
  /* Initialize LEDs mounted on STM8L152X-EVAL board */
  GPIO_Init(LED_GPIO_PORT, LED_GPIO_PINS, GPIO_Mode_Out_PP_Low_Fast);
  SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA, ENABLE);

  //USART1 CLK enable
	CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
  GPIO_ExternalPullUpConfig(GPIOA, GPIO_Pin_3, ENABLE);
  GPIO_ExternalPullUpConfig(GPIOA, GPIO_Pin_4, ENABLE);

  //Enable UART TX
  USART_Init(USART1, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, (USART_Mode_Rx|USART_Mode_Tx));
  USART_ClockInit(USART1, USART_Clock_Disable, USART_CPOL_Low, USART_CPHA_1Edge, USART_LastBit_Disable);
  USART_Cmd(USART1, ENABLE);

  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
  I2C_Init(I2C1, I2C_SPEED, 0xA0, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
  I2C_Cmd(I2C1, ENABLE);

  GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_OD_HiZ_Fast);
  GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_OD_HiZ_Fast);


  i2c_write(0);

  // tiny_print(greeting, ARR_SIZE(greeting));
  // tiny_scan(response, ARR_SIZE(response));
  // tiny_print(response, ARR_SIZE(response));
  while (1)
  {

    //tiny_print(greeting, ARR_SIZE(greeting));

    i2c_resp = read_rtc();
    time_dec = decode_rtc(i2c_resp);

    if (time_dec != time_dec_last)
    {
      time_dec_last = time_dec;
      print_secs(i2c_resp);
    }


    GPIO_ToggleBits(LED_GPIO_PORT, LED_GPIO_PINS);
    Delay(0xFF00);
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint16_t nCount)
{
  /* Decrement nCount value */

	while (nCount != 0)
  {
    nCount--;
  }
	
}

char putchar (char c)
{
  /* Write a character to the UART1 */
  USART_SendData8(USART1, c);
/* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
  {
  }
  return (c);
}

char getchar (void)
{

  char c = 0;
  /* Loop until the Read data register flag is SET */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
  {
  }
  c = USART_ReceiveData8(USART1);
  return (c);
}

/**
 * @brief Print pre-formatted buffer via UART
 *
*/
void tiny_print (char* str, int len)
{
  int i;

  for (i=0; i<len; i++)
  {
    putchar(str[i]);
  }

  return;
}

void tiny_scan (char* out, int len)
{
  int i;

  for (i=0; i<len; i++)
  {
    out[i] = getchar();
  }

  return;
}

void i2c_write(uint8_t data)
{
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1, 0b11010000, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1, data);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_SendData(I2C1, 0b00000000);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(I2C1, ENABLE);

  return;
}

uint8_t read_rtc(void)
{

  uint8_t i2c_resp = 0;

  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1, 0b11010000, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C1, 0);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  I2C_Send7bitAddress(I2C1, 0b11010000, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  I2C_AcknowledgeConfig(I2C1, DISABLE);

  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
  i2c_resp = I2C_ReceiveData(I2C1);

  I2C_GenerateSTOP(I2C1, ENABLE);

  return i2c_resp;
}

uint8_t decode_rtc(uint8_t val)
{
  uint8_t ten_ticks = ((val & TEN_SEC_BITMASK) >> 4) * 10;
  uint8_t ticks = val & SECONDS_BITMASK;
  return ten_ticks + ticks;
}

void print_secs(uint8_t secs)
{
  const char header[] = "Seconds: ";
  const char newline[] = "\r\n";
  char out[2];
  uint8_t tens = (uint8_t)(secs >> 4) + 48;
  uint8_t ones = (uint8_t)(secs & SECONDS_BITMASK) + 48;
  out[0] = tens;
  out[1] = ones;
  tiny_print(header, ARR_SIZE(header));
  tiny_print(out, ARR_SIZE(out));
  tiny_print(newline, ARR_SIZE(newline));

  return;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

