#include <Arduino.h>
#include <SimpleFOC.h>

extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
#if defined(RCC_PLLR_SUPPORT)
  RCC_OscInitStruct.PLL.PLLR = 2;
#endif

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1) {}
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    while (1) {}
  }

  // Make sure the global clock variable is updated
  SystemCoreClockUpdate();
}

HardwareSerial SerialDebug(USART3);

/**
 * Should output the following:
 *  
 * SYS_CLK: 96000000
 * PCLK1: 48000000
 * PCLK2: 96000000

 */

 
static void printClockInfo() {
  SerialDebug.println("----- Clock Info -----");

  SerialDebug.print("HSE_VALUE: ");
  SerialDebug.println(HSE_VALUE);

  SerialDebug.print("F_CPU: ");
  SerialDebug.println(F_CPU);

  SerialDebug.print("SystemCoreClock: ");
  SerialDebug.println(SystemCoreClock);

  SerialDebug.print("SYSCLK: ");
  SerialDebug.println(HAL_RCC_GetSysClockFreq());

  SerialDebug.print("HCLK: ");
  SerialDebug.println(HAL_RCC_GetHCLKFreq());

  SerialDebug.print("PCLK1: ");
  SerialDebug.println(HAL_RCC_GetPCLK1Freq());

  SerialDebug.print("PCLK2: ");
  SerialDebug.println(HAL_RCC_GetPCLK2Freq());

  SerialDebug.println("----------------------");
}

void enablePowerRails() {
  digitalWrite(PB9, HIGH);
  delay(1000);
  digitalWrite(PB4, HIGH);
}

void setup() {
  SerialDebug.setTx(PC10);
  SerialDebug.setRx(PC11);
  delay(3000);
  SerialDebug.begin(115200);

  pinMode(PB9, OUTPUT);
  pinMode(PB4, OUTPUT);
  
  delay(200);

  enablePowerRails();

  SerialDebug.println();
  SerialDebug.println("Clock + Arduino + SimpleFOC test");

  printClockInfo();

  if (HAL_RCC_GetSysClockFreq() == 96000000UL) {
    SerialDebug.println("SYSCLK is correct: 96 MHz");
  } else {
    SerialDebug.println("SYSCLK is NOT 96 MHz");
  }
}

void loop() {
  static uint32_t last = 0;

  if (millis() - last >= 1000) {
    last = millis();
    SerialDebug.println("running");
  }
}