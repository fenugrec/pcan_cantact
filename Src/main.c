#include "stm32f0xx_hal.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"
#include "pcan_protocol.h"
#include "pcan_usb.h"

void HAL_MspInit( void )
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

#ifdef EXTERNAL_CLOCK
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  /* HSE = 16MHZ */
#if EXTERNAL_CLOCK == 16
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  /* HSE = 8MHZ */
#elif EXTERNAL_CLOCK == 8
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
#else
#error invalid HSE_VALUE
#endif
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 );

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit );
}
#else
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}
#endif

void SysTick_Handler( void )
{
  HAL_IncTick();
}

void cannette_init(void) {
#if BOARD==cannette
	#define nCANSTBY_Port GPIOC
	#define nCANSTBY_Pin GPIO_PIN_14	/* control xceiver standby, active low */

	#define nSI86EN_Port GPIOC
	#define nSI86EN_Pin GPIO_PIN_13		/* enable power to Si86xx isolater, active low */

	#define DCDCEN_Port GPIOC
	#define DCDCEN_Pin GPIO_PIN_15		/* activate DCDC converter, active high */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = nCANSTBY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(nCANSTBY_Port, &GPIO_InitStruct);	//xceiver active.

	HAL_GPIO_WritePin(DCDCEN_Port, DCDCEN_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = DCDCEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DCDCEN_Port, &GPIO_InitStruct);	//start DCDC (TODO : wait until enumerated)

	HAL_GPIO_WritePin(nSI86EN_Port, nSI86EN_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = nSI86EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(nSI86EN_Port, &GPIO_InitStruct);	//enable si86
#endif
}

int main( void )
{
  HAL_Init();

  SystemClock_Config();
  cannette_init();

  pcan_usb_init();
  pcan_led_init();
  pcan_timestamp_init();
  pcan_protocol_init();

  pcan_led_set_mode( LED_CH0_RX, LED_MODE_BLINK_SLOW, 0 );
  pcan_led_set_mode( LED_CH0_TX, LED_MODE_BLINK_SLOW, 0 );

  for(;;)
  {
    pcan_usb_poll();
    pcan_led_poll();
    pcan_protocol_poll();
  }
}
