#ifndef		__N6_LCD_H__
#define		__N6_LCD_H__

/*******************************    INCLUDES   ********************************/
#include "main.h"
                 // Device header


/***************************    GLOBAL VARIABLES   ****************************/

extern struct N6_RTOS_struct_def N6;
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

/***************************       FUNCTIONS       ****************************/

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);

void N6_LCD_Innit(void);

#endif