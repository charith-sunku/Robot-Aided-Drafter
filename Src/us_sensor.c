/**
 * @file us_sensor.c
 * @brief Ultrasonic distance sensor (HC‑SR04) driver utilities.
 *
 * @date 2025-06-12
 * @author Dylan Featherson, Tomas Franco, Charith Sunku
 */
#include "us_sensor.h"
#include "stm32f4xx_hal.h"
/**
 * @brief  Initializes ultrasonic sensor
 *  
 */

void sensorInit(void) {
    // TRIG = output
    GPIO_InitTypeDef g = {0};
    g.Pin   = US_TRIG_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(US_TRIG_GPIO_Port,&g);
    // ECHO = input
    g.Pin   = US_ECHO_Pin;
    g.Mode  = GPIO_MODE_INPUT;
    HAL_GPIO_Init(US_ECHO_GPIO_Port,&g);
}
/**
 * @brief  Returns distance read by ultrasonic sensor
 *  
 */

float ultrasonicDistance(void) {
    // trigger 10µs pulse
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port,US_TRIG_Pin,GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port,US_TRIG_Pin,GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(US_TRIG_GPIO_Port,US_TRIG_Pin,GPIO_PIN_RESET);

    // wait for ECHO high
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port,US_ECHO_Pin)==GPIO_PIN_RESET
           && (HAL_GetTick()-t0)<100);
    uint32_t start = DWT->CYCCNT;

    // wait for ECHO low
    while (HAL_GPIO_ReadPin(US_ECHO_GPIO_Port,US_ECHO_Pin)==GPIO_PIN_SET
           && (HAL_GetTick()-t0)<100);
    uint32_t end = DWT->CYCCNT;

    float dt_us = (end - start) * (1e6f / HAL_RCC_GetHCLKFreq());
    // speed of sound ≈ 0.343 mm/µs, round-trip
    return dt_us * 0.343f / 2.0f;
}
/**
 * @brief  Returns TRUE if ultrasonic distance is too small (ie. object detected)
 *  
 */

bool obstruction(void) {
    float d = ultrasonicDistance();
    return (d > 0 && d < 35.0f);  // e.g. <20 mm
}


