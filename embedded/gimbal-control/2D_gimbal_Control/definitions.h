#include <Arduino.h>

#ifndef MY_HEADER_H // Header guard
#define MY_HEADER_H

/**
MOTOR SPECIFICATIONS
*/

#define POLE_PAIRS 7
#define PHASE_RESISTANCE 2.3f
#define KV_RATING 220
#define Q_INDUCTANCE 0.00086f


/**
MOTOR PINTOUTS
*/

#define B_PH1 1
#define B_PH2 2
#define B_PH3 3
#define B_EN  5

#define T_PH1 6
#define T_PH2 7
#define T_PH3 8
#define T_EN  5

/**
POWER TELEMETRY PINOUTS
*/

#define V_12 
#define V_5
#define V_3
#define VB_PH1
#define VB_PH2
#define VB_PH3
#define VT_PH1
#define VT_PH2
#define VT_PH3
#define GAIN

/**
PRIVATE DEFINES
*/
#define EXT_INT_Pin GPIO_PIN_13
#define EXT_INT_GPIO_Port GPIOC
#define YAW_nFAULT_Pin GPIO_PIN_14
#define YAW_nFAULT_GPIO_Port GPIOC
#define YAW_EN_Pin GPIO_PIN_15
#define YAW_EN_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define ADC_5V_Pin GPIO_PIN_0
#define ADC_5V_GPIO_Port GPIOC
#define ADC_12V_Pin GPIO_PIN_1
#define ADC_12V_GPIO_Port GPIOC
#define ADC_3_3V_Pin GPIO_PIN_2
#define ADC_3_3V_GPIO_Port GPIOC
#define ADC_PITCH_RS3_Pin GPIO_PIN_3
#define ADC_PITCH_RS3_GPIO_Port GPIOC
#define ADC_eFUSE_V_Pin GPIO_PIN_0
#define ADC_eFUSE_V_GPIO_Port GPIOA
#define ADC_eFUSE_I_Pin GPIO_PIN_1
#define ADC_eFUSE_I_GPIO_Port GPIOA
#define STM_TX_CM_RX_Pin GPIO_PIN_2
#define STM_TX_CM_RX_GPIO_Port GPIOA
#define STM_RX_CM_TX_Pin GPIO_PIN_3
#define STM_RX_CM_TX_GPIO_Port GPIOA
#define ADC_YAW_RS1_Pin GPIO_PIN_4
#define ADC_YAW_RS1_GPIO_Port GPIOA
#define ADC_YAW_RS2_Pin GPIO_PIN_5
#define ADC_YAW_RS2_GPIO_Port GPIOA
#define ADC_YAW_RS3_Pin GPIO_PIN_6
#define ADC_YAW_RS3_GPIO_Port GPIOA
#define YAW_IN1_Pin GPIO_PIN_7
#define YAW_IN1_GPIO_Port GPIOA
#define ADC_PITCH_RS2_Pin GPIO_PIN_4
#define ADC_PITCH_RS2_GPIO_Port GPIOC
#define ADC_PITCH_RS1_Pin GPIO_PIN_5
#define ADC_PITCH_RS1_GPIO_Port GPIOC
#define YAW_IN2_Pin GPIO_PIN_0
#define YAW_IN2_GPIO_Port GPIOB
#define YAW_IN3_Pin GPIO_PIN_1
#define YAW_IN3_GPIO_Port GPIOB
#define YAW_nSLEEP_Pin GPIO_PIN_2
#define YAW_nSLEEP_GPIO_Port GPIOB
#define YAW_SCL_Pin GPIO_PIN_10
#define YAW_SCL_GPIO_Port GPIOB
#define YAW_nRESET_Pin GPIO_PIN_12
#define YAW_nRESET_GPIO_Port GPIOB
#define PITCH_EN_Pin GPIO_PIN_13
#define PITCH_EN_GPIO_Port GPIOB
#define PITCH_nFAULT_Pin GPIO_PIN_14
#define PITCH_nFAULT_GPIO_Port GPIOB
#define PITCH_nSLEEP_Pin GPIO_PIN_15
#define PITCH_nSLEEP_GPIO_Port GPIOB
#define PITCH_IN1_Pin GPIO_PIN_6
#define PITCH_IN1_GPIO_Port GPIOC
#define PITCH_IN2_Pin GPIO_PIN_7
#define PITCH_IN2_GPIO_Port GPIOC
#define PITCH_IN3_Pin GPIO_PIN_8
#define PITCH_IN3_GPIO_Port GPIOC
#define CM_SDA_Pin GPIO_PIN_9
#define CM_SDA_GPIO_Port GPIOC
#define CM_SCL_Pin GPIO_PIN_8
#define CM_SCL_GPIO_Port GPIOA
#define CM_TO_STM_Pin GPIO_PIN_9
#define CM_TO_STM_GPIO_Port GPIOA
#define STM_TO_CM_Pin GPIO_PIN_10
#define STM_TO_CM_GPIO_Port GPIOA
#define USB_N_Pin GPIO_PIN_11
#define USB_N_GPIO_Port GPIOA
#define USB_P_Pin GPIO_PIN_12
#define USB_P_GPIO_Port GPIOA
#define STM_SWDIO_Pin GPIO_PIN_13
#define STM_SWDIO_GPIO_Port GPIOA
#define STM_SWCLK_Pin GPIO_PIN_14
#define STM_SWCLK_GPIO_Port GPIOA
#define STM_STAT_Pin GPIO_PIN_15
#define STM_STAT_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOC
#define DEBUG_RX_Pin GPIO_PIN_11
#define DEBUG_RX_GPIO_Port GPIOC
#define PG_3_3V_Pin GPIO_PIN_12
#define PG_3_3V_GPIO_Port GPIOC
#define PG_12V_Pin GPIO_PIN_2
#define PG_12V_GPIO_Port GPIOD
#define YAW_SDA_Pin GPIO_PIN_3
#define YAW_SDA_GPIO_Port GPIOB
#define EN_12V_Pin GPIO_PIN_4
#define EN_12V_GPIO_Port GPIOB
#define PITCH_nRESET_Pin GPIO_PIN_5
#define PITCH_nRESET_GPIO_Port GPIOB
#define PITCH_SCL_Pin GPIO_PIN_6
#define PITCH_SCL_GPIO_Port GPIOB
#define PITCH_SDA_Pin GPIO_PIN_7
#define PITCH_SDA_GPIO_Port GPIOB
#define PG_5V_Pin GPIO_PIN_8
#define PG_5V_GPIO_Port GPIOB
#define EN_5V_Pin GPIO_PIN_9
#define EN_5V_GPIO_Port GPIOB


#endif // MY_HEADER_H



