#include "Arduino.h"
#include "PeripheralPins.h"

// Strong override of the WEAK PinMap_PWM in the variant file.
// ONLY the pins listed here will work for PWM. Add every PWM pin you use.
const PinMap PinMap_PWM[] = {
    // --- Motor 1 on TIM3 ---
    {PA_7, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 2, 0)}, // TIM3_CH2
    {PB_0, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 3, 0)}, // TIM3_CH3
    {PB_1, TIM3, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 4, 0)}, // TIM3_CH4

    // --- Motor 2 on TIM8 ---
    {PC_6, TIM8, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 1, 0)}, // TIM8_CH1
    {PC_7, TIM8, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 2, 0)}, // TIM8_CH2
    {PC_8, TIM8, STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 3, 0)}, // TIM8_CH3

    {NC, NP, 0}
};
