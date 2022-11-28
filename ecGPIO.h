/**
******************************************************************************
* @author  HeoDabin
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13

#define EC_NUD 0x00
#define EC_PU 0x01
#define EC_PD 0x10
#define EC_RV 0x11

#define Low 0x00
#define Medium 0x01
#define Fast 0x10
#define High 0x11

#define Push_Pull 0
#define Open_Drain 1

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void sevensegment_init(void);
void LED_init(void);
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);
void led_decode(int num);
void sevensegment_decode(int num);
void LED_toggle(void);
void LED_toggle2(GPIO_TypeDef *Port, int pin);
void LED_init_test(void);
void sevensegment_init_test(void);
void sevensegment_decode_test(int num);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
