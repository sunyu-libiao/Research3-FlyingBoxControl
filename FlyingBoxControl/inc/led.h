/**
  ******************************************************************************
  * @file    led.h
  * @brief   板载 LED（PB9）驱动接口
  ******************************************************************************
  */

#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

void LED_Init(void);
void LED_TaskCreate(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
