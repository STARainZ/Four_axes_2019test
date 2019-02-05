/*
 * colorful_LED.h
 *
 *  Created on: 2017Äê10ÔÂ6ÈÕ
 *      Author: 79864
 */

#ifndef COLORFUL_LED_COLORFUL_LED_H_
#define COLORFUL_LED_COLORFUL_LED_H_

void LED_ColorInit(void);
void LED_Color(uint8_t r,uint8_t g,uint8_t b);
void LED_Config(void);
void LED_Set(uint8_t color);
void Led_Twinkle(uint8_t color,uint8_t times);

#define RED 0
#define BLUE 1
#define GREEN 2
#define MAGENTA 3
#define YELLOW 4
#define LIGHTBLUE 5
#define WHITE 6
#define LedOff 7

#endif /* COLORFUL_LED_COLORFUL_LED_H_ */
