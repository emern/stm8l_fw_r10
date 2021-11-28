/**
 * @file main.c
 * @brief Main implementation for RTC based timer (print over USB)
 */

/* Includes */
#include "stm8l15x.h"
#include <string.h>

/* Local defines */

/* Blinky LED address information */
#define LED_GPIO_PORT  GPIOA
#define LED_GPIO_PINS  GPIO_Pin_4

/* I2C config data */
#define I2C_SPEED 100000

/* Useful macro for reducing runtime code size */
#define ARR_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* RTC bitmasks */
#define TEN_SEC_BITMASK 0b01110000
#define SECONDS_BITMASK 0b00001111

/* RTC data payload read size */
#define RTC_PAY_READ_SIZE 2

/* Output data string buffer size */
#define OUTPUT_STR_BUF_SIZE 3

/* Local enums and structs */

typedef enum
{
  RTC_PRINT_SECONDS,

  RTC_PRINT_MINUTES,

  RTC_PRINT_HOURS,

  RTC_PRINT_DAYS,

  RTC_PRINT_MONTHS,

  RTC_PRINT_YEARS

} print_type_t;

/* Private function prototypes */
void Delay (uint16_t nCount);
char putchar (char c);
char getchar (void);
void tiny_print (char* str, int len);
void tiny_scan (char* out, int len);
void rtc_write(uint8_t addr, uint8_t data);
void read_rtc(uint8_t* bytes, uint8_t len);
uint8_t decode_rtc(uint8_t val);
void print_rtc_val(uint8_t val, print_type_t type);


void main(void)
{
  /* Initialize system variables */
  uint8_t i2c_data[] = {0,0};
  uint8_t secs_dec = 0;
  uint8_t secs_dec_last = 0;

  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_2);
  /* Initialize LEDs mounted on STM8L152X-EVAL board */
  GPIO_Init(LED_GPIO_PORT, LED_GPIO_PINS, GPIO_Mode_Out_PP_Low_Fast);
  SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA, ENABLE);

  /* USART1 CLK enable */
	CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
  GPIO_ExternalPullUpConfig(GPIOA, GPIO_Pin_3, ENABLE);
  GPIO_ExternalPullUpConfig(GPIOA, GPIO_Pin_4, ENABLE);

  /* Enable UART TX */
  USART_Init(USART1, 115200, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, (USART_Mode_Rx|USART_Mode_Tx));
  USART_ClockInit(USART1, USART_Clock_Disable, USART_CPOL_Low, USART_CPHA_1Edge, USART_LastBit_Disable);
  USART_Cmd(USART1, ENABLE);

  /* Enable I2C module */
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
  I2C_Init(I2C1, I2C_SPEED, 0xA0, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
  I2C_Cmd(I2C1, ENABLE);

  /* Configure I2C GPIO to HiZ (board has external 10k pullups) */
  GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_OD_HiZ_Fast);
  GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_OD_HiZ_Fast);

  /* Must write 0 to RTC to disable the Clock Halt bit and clear data stored in NV (seconds) */
  rtc_write(0, 0);
  /* Clear NV (minutes) */
  rtc_write(1, 0);

  /* Main loop */
  while (1)
  {

    /* Read rtc data */
    read_rtc(i2c_data, RTC_PAY_READ_SIZE);
    /* Decode number of seconds recieved (always first byte) to determine if print is needed */
    secs_dec = decode_rtc(i2c_data[0]);

    /* Print time data if new second has rolled over */
    if (secs_dec != secs_dec_last)
    {
      secs_dec_last = secs_dec;
      print_rtc_val(i2c_data[0], RTC_PRINT_SECONDS);
      print_rtc_val(i2c_data[1], RTC_PRINT_MINUTES);
    }

    /* Toggle on board GPIO and delay */
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

/**
 * @brief Send byte to host via UART
 * @param c: Byte to send
 * @retval Byte sent
*/
char putchar (char c)
{
  /* Write a character to the UART1 */
  USART_SendData8(USART1, c);
/* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  return (c);
}

/**
 * @brief Recieve byte from host via UART
 * @retval Byte recieved
 * @note This operation is blocking, if host does not send any data, device will poll indefinitely
*/
char getchar (void)
{

  char c = 0;
  /* Loop until the Read data register flag is SET */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
  c = USART_ReceiveData8(USART1);
  return (c);
}

/**
 * @brief Print pre-formatted string via UART
 * @param str: Pre-formatted string to print
 * @param len: Length of string INCLUDING NULL TERMINATOR
 * @retval None
 *
 * @note In order to reduce compiled code size, the ARR_SIZE macro is reccomended
 *       to be used to calculate the len parameter at compile time. Since ARR_SIZE
 *       calculates complete buffer size, NULL terminator is expected in the value
 *       of len.
*/
void tiny_print (char* str, int len)
{
  int i;

  /* Double check string length */
  if (strlen(str) != len - 1)
  {
    return;
  }

  /* Send string via UART */
  for (i=0; i<len; i++)
  {
    putchar(str[i]);
  }
}

/**
 * @brief Scan for string of length len via UART
 * @param out: Data buffer containing recieved string
 * @param len: Length of expected string INCLUDING NULL TERMINATOR
 * @retval None
 *
 * @note In order to reduce compiled code size, the ARR_SIZE macro is reccomended
 *       to be used to calculate the len parameter at compile time. Since ARR_SIZE
 *       calculates complete buffer size, NULL terminator is expected in the value
 *       of len.
*/
void tiny_scan (char* out, int len)
{
  int i;

  if (strlen(out) != len - 1)
  {
    return;
  }

  for (i=0; i<len; i++)
  {
    out[i] = getchar();
  }
}

/**
 * @brief Write byte to RTC register addr
 * @param addr: RTC register address to write data byte to
 * @param data: Data to write
 * @retval None
*/
void rtc_write(uint8_t addr, uint8_t data)
{
  /* Generate I2C start condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send RTC device address */
  I2C_Send7bitAddress(I2C1, 0b11010000, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send target register address */
  I2C_SendData(I2C1, addr);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send data byte */
  I2C_SendData(I2C1, data);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Generate stop condition */
  I2C_GenerateSTOP(I2C1, ENABLE);

  return;
}


/**
 * @brief Transmit starting address and then act as master reciever
 *
 * Example on multibyte data reads avaliable in section 28.4.2 (fig 147) of Doc # RM0031 STM8L TRM
*/
void read_rtc(uint8_t* bytes, uint8_t len)
{
  uint8_t i;

  /* Generate I2C start event, (EV5) */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send slave device address, (EV6) */
  I2C_Send7bitAddress(I2C1, 0b11010000, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send register read address, in this case we start read from RTC base register (address 0) (EV_8) */
  I2C_SendData(I2C1, 0);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Generate repeated start condition (EV5) */
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send slave device address, (EV6) */
  I2C_Send7bitAddress(I2C1, 0b11010000, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* Not actually in the datasheet but we must explicitly enable I2C ACK in this stage */
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  /* If only a single byte is being read, prepare the stop condition */
  if (len == 1)
  {
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
  }

  /* Read i bytes from RTC, on second to last data read we must set a NACK and set a STOP condition */
  for (i=0; i<len; i++)
  {
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    bytes[i] = I2C_ReceiveData(I2C1);

    /* Case for if i = 1 already handled by pre-loop statement */
    if (i == (len - 2))
    {
      /* Disable ACK to generate final NACK and STOP conditions */
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      I2C_GenerateSTOP(I2C1, ENABLE);
    }
  }
}

/**
 * @brief Function to decode BCD RTC value to decimal
 * @param val: Raw encoded value
 * @retval Decoded decimal time value
*/
uint8_t decode_rtc(uint8_t val)
{
  uint8_t ten_ticks = (val >> 4) * 10;
  uint8_t ticks = val & SECONDS_BITMASK;
  return ten_ticks + ticks;
}

/**
 * @brief Function to print BCD encoded RTC value to Host PC
 * @param val: Encoded RTC value
 * @param type: Data type indicator
*/
void print_rtc_val(uint8_t val, print_type_t type)
{
  char out[3];
  const char secs_header[] = "Seconds: ";
  const char mins_header[] = "Minutes: ";
  const char newline[] = "\r\n";
  const char line_break[] = "\r\n\r\n";

  /**
   * Convert BCD to ASCII by bit manipulations.
   *
   * Step 1) Extract "10's" digit by recovering only bits 4:6 of the RTC value to out[0]
   * Step 2) Extract "1's" digit by bitmasking the lower 4 bits of RTC value to out[1]
   * Step 3) Convert decimal digits to equivalent ASCII interpretation by adding constant 48
   *
   * Note: tiny_print expects NULL terminated string, must append NULL character for databuffer
  */
  out[0] = (char)((val >> 4) + 48);
  out[1] = (char)((val & SECONDS_BITMASK) + 48);
  out[2] = '\0';

  /* Print based on type */
  switch (type)
  {
    case RTC_PRINT_SECONDS:
      tiny_print(secs_header, ARR_SIZE(secs_header));
      tiny_print(out, OUTPUT_STR_BUF_SIZE);
      tiny_print(newline, ARR_SIZE(newline));
      break;

    case RTC_PRINT_MINUTES:
      tiny_print(mins_header, ARR_SIZE(mins_header));
      tiny_print(out, OUTPUT_STR_BUF_SIZE);
      tiny_print(line_break, ARR_SIZE(line_break));
      break;

    default:
      break;
  }
}


