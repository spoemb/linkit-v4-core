#ifndef RSPB_V1_H
#define RSPB_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LED definitions for RSPB V1
#define LEDS_NUMBER    4

#define LED2_R         NRF_GPIO_PIN_MAP(1,7)
#define LED2_G         NRF_GPIO_PIN_MAP(1,10)
#define LED2_B         NRF_GPIO_PIN_MAP(1,4)
#define LED2_POWER_CONTROL  NRF_GPIO_PIN_MAP(0,4)

#define LED_1          LED2_R
#define LED_2          LED2_G
#define LED_3          LED2_B
#define POWER_CONTROL  LED2_POWER_CONTROL

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1, LED_2, LED_3, POWER_CONTROL }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      POWER_CONTROL

#define BUTTONS_NUMBER 1

#define BUTTON_1       NRF_GPIO_PIN_MAP(0,11)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define HWFC           true

#ifdef __cplusplus
}
#endif

#endif // RSPB_V1_H
