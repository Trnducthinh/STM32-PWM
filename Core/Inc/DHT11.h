#ifndef __DHT11_H__
#define __DHT11_H__

#include "stm32f1xx_hal.h"  // Ho?c dúng dòng MCU b?n dùng

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1  // Ch?nh dúng pin b?n k?t n?i

void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);
void DHT11_Read_Data(void);

extern float tCelsius;
extern float tFahrenheit;
extern float RH;

#endif /* __DHT11_H__ */
