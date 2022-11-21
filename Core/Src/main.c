/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * CONEXIONES:
 * HC-SR04 (PROXIMIDAD): Vcc = +5V, Trig = PA9, Echo = PA8
 * SSD1306 (OLED): Vcc = +3.3V, SDA = PB7, SCL = PB6
 * HUMEDAD: Vcc = +5V, AO = PC4
 * DHT11 (TEMPERATURA): Vcc = +5V, DAT = A1
 * PA4: Entrada botón
 * PA6: Salida relé para motobomba
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"
#include "planta.h"
#include "delay.h"
#include "app_nucleo_l476.h"
#include "animation.h"
#include "distancia.h"
#include "temperatura.h"
#include "humedad.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


char strCopy[30]; //Para escribir en el OLED


uint8_t Button = 0; //BOMBA

extern void FSM_Read_PushButton_1(uint8_t *flag);//BOMBA
extern void DireccionalDerecha(uint8_t *flag);//BOMBA
static uint8_t temp_1 = 0;//BOMBA

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc1); //Para humedad
	HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low (ULTRASONICO)
	SSD1306_Init();


	SSD1306_GotoXY (0, 0);
	SSD1306_Puts ("INICALIZANDO...", &Font_7x10, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(1000);
	SSD1306_Clear();


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		static uint64_t c;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////SENSOR DISTANCIA ULTRASÓNICO////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		DISTANCIA();



//ACTIVACION MANUAL CON BOTON
		if(Distance < 13){
			FSM_Read_PushButton_1(&temp_1); //LECTURA BOTÓN
			DireccionalDerecha(&temp_1); //ACTIVACION MANUAL BOMBA
			if(temp_1==1){
			c=0;
			while(temp_1==1 && Distance < 13){
				if(c++ == 0){ //Para limpiar display una sola vez
				SSD1306_Clear();}
				animation1(); //Animacion para modo manual
				FSM_Read_PushButton_1(&temp_1);
				DISTANCIA(); //Para salir del loop por si se acaba el agua en el modo manual
			}
			LD3_Reset();
			SSD1306_Clear();
		}}





		//IMPRESIÓN EN LCD DISTANCIA
		SSD1306_GotoXY (0, 0);
		sprintf(strCopy,"%dcm ", Distance);
		SSD1306_Puts (strCopy, &Font_11x18, 1);
		SSD1306_UpdateScreen();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////RECARGA BOMBA///////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//AVISO LLENADO DE BOMBA, CALIBRAR - MUESTRA DATOS PERO NO ACCIONA LA BOMBA DE AGUA
		if(Distance >= 13){
			c=0;
			while(Distance >= 13){
				if(c++ == 0){
					SSD1306_Clear(); //HACE UN CLEAR DEL DISPLAY SOLO LA PRIMERA VEZ PARA QUITAR LAS LETRAS
					SSD1306_Fill(SSD1306_COLOR_WHITE);
					SSD1306_GotoXY (0, 0);
					sprintf(strCopy,"RECARGAR");
					SSD1306_Puts (strCopy, &Font_11x18, 0);
					SSD1306_GotoXY (0, 18);
					sprintf(strCopy,"TANQUE");
					SSD1306_Puts (strCopy, &Font_11x18, 0);
					SSD1306_UpdateScreen();
				}

			//LECTURAS SIN ACCIONAR BOMBA//

				//TEMPERATURA
				TEMPERATURA();
				sprintf(strCopy,"%d.%dC", TCI, TCD);
				SSD1306_GotoXY (66, 36);
				SSD1306_Puts (strCopy, &Font_7x10, 0);
				SSD1306_UpdateScreen();

				//DISTANCIA

				SSD1306_GotoXY (0, 36);
				sprintf(strCopy,"%dcm ", Distance);
				SSD1306_Puts (strCopy, &Font_7x10, 0);
				SSD1306_UpdateScreen();

				//HUMEDAD
				HUMEDAD();

				if (readValue > 3200)
				{
					SSD1306_GotoXY (0, 46);
					sprintf(strCopy,"Humedad baja ");
					SSD1306_Puts (strCopy, &Font_7x10, 0);

				}
				else if (readValue > 1500 )
				{
					SSD1306_GotoXY (0, 46);
					sprintf(strCopy,"Humedad media ");
					SSD1306_Puts (strCopy, &Font_7x10, 0);
					SSD1306_UpdateScreen();

				}
				else
				{
					SSD1306_GotoXY (0, 46);
					sprintf(strCopy,"Humedad alta ");
					SSD1306_Puts (strCopy, &Font_7x10, 0);
					SSD1306_UpdateScreen();
				}

			//PARA SALIR DEL LOOP:
				DISTANCIA();

			}
			SSD1306_Fill(SSD1306_COLOR_BLACK);
		}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////SENSOR TEMPERATURA DHT11/////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		TEMPERATURA();
		HUMEDAD(); //Diagrama de flujo, es por si no entré a ningun ciclo anterior, nunca habré leido la temperatura

		//IMPRESIÓN EN LCD TEMPERATURA
		sprintf(strCopy,"%d.%dC", TCI, TCD);
		SSD1306_GotoXY (65, 0);
		SSD1306_Puts (strCopy, &Font_11x18, 1);
		SSD1306_UpdateScreen();



		//Accionamiento sensor
		if(TCI >= 26 && Distance < 13) //El distance es para asegurarse que haya agua en la bomba, es solo protocolo de seguridad ya que el sistema espera a que haya agua en el tanque
		{
			SSD1306_GotoXY (0, 40);
			sprintf(strCopy,"Temperatura alta ");
			SSD1306_Puts (strCopy, &Font_7x10, 1);
			SSD1306_GotoXY (0, 50);

			if(readValue > 2400){ //Que no riegue si hay humedad alta, solo media o menor, la diferencia con la humedad baja aqui es que mostrara la temperatura que es un dato importante en este caso
			sprintf(strCopy,"Iniciando riego...");
			SSD1306_Puts (strCopy, &Font_7x10, 1);
			SSD1306_UpdateScreen();
			HAL_Delay(1000);
			c=0;

			while (TCI >= 26 && Distance < 13 && readValue > 1500) //Aqui si es importante el distance para no dañar la bomba, el readValue es para la humedad, no tiene sentido regar las plantas con humedad muy alta
			{

				if(c++ == 0){
					LD3_Set(); //ACTIVA LA BOMBA LA PRIMERA VEZ QUE ENTRE AL LOOP.
					SSD1306_Clear(); //HACE UN CLEAR DEL DISPLAY SOLO LA PRIMERA VEZ PARA QUITAR LAS LETRAS
				}

				//BEGIN PLANT ANIMATION
				animation1();

				sprintf(strCopy,"%d.%dC", TCI, TCD); //Para mostrar temperatura mientras riega
				SSD1306_GotoXY (0, 0);
				SSD1306_Puts (strCopy, &Font_11x18, 1);
				SSD1306_UpdateScreen();

				//END PLANT ANIMATION

				//Para salir del ciclo:
				TEMPERATURA();

				//MEDIDA DE DISTANCIA
				DISTANCIA();


				//MEDIDA DE HUMEDAD
				HUMEDAD();
			}
			SSD1306_Clear();
			SSD1306_UpdateScreen();
			LD3_Reset(); //Apagar bomba despues del ciclo
		}}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////SENSOR HUMEDAD EN LA TIERRA//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		HUMEDAD();

		//Implementacion sensor
		if (readValue > 3200 && Distance < 13) //El distance es para asegurarse que haya agua en la bomba, es solo protocolo de seguridad ya que el sistema espera a que haya agua en el tanque
		{
			SSD1306_GotoXY (0, 30);
			sprintf(strCopy,"Humedad baja ");
			SSD1306_Puts (strCopy, &Font_7x10, 1);
			SSD1306_GotoXY (0, 50);
			sprintf(strCopy,"Iniciando riego... ");
			SSD1306_Puts (strCopy, &Font_7x10, 1);
			SSD1306_UpdateScreen();
			HAL_Delay(1000);
			c=0;
			while (readValue > 3200 && Distance < 13) //Aqui es importante el distance para no dañar la bomba)
			{

				if(c++ == 0){
					LD3_Set(); //ACTIVA LA BOMBA LA PRIMERA VEZ QUE ENTRE AL LOOP.
					SSD1306_Clear(); //HACE UN CLEAR DEL DISPLAY SOLO LA PRIMERA VEZ PARA QUITAR LAS LETRAS
				}

				//BEGIN PLANT ANIMATION
				animation1();
				//END PLANT ANIMATION

				HUMEDAD(); //Para salir del ciclo
				DISTANCIA(); //Para la bomba


			}

			SSD1306_Clear();
			SSD1306_UpdateScreen();

			LD3_Reset(); //DESACTIVA LA BOMBA DESPUÉS FINAL DEL WHILE, HAY QUE PONERLA AQUI PARA QUE NO SE PARE EN PROCESO MANUAL (BOTON) CUANDO DETECTE OTRAS HUMEDADES
		}
		else if (readValue > 1500 )
		{
			SSD1306_GotoXY (0, 30);
			sprintf(strCopy,"Humedad media ");
			SSD1306_Puts (strCopy, &Font_7x10, 1);
			SSD1306_UpdateScreen();

			if(TCI < 26){
				SSD1306_GotoXY (0, 50);
				sprintf(strCopy,"                  "); //Para borrar iniciando regado SOLO cuando no hay altas temperaturas
				SSD1306_Puts (strCopy, &Font_7x10, 1);
				SSD1306_UpdateScreen();
			}}
		else
		{
			SSD1306_GotoXY (0, 30);
			sprintf(strCopy,"Humedad alta ");
			SSD1306_Puts (strCopy, &Font_7x10, 1);
			SSD1306_UpdateScreen();

			if(TCI < 26){
				SSD1306_GotoXY (0, 50);
				sprintf(strCopy,"                  "); //Para borrar iniciando regado SOLO cuando no hay altas temperaturas
				SSD1306_Puts (strCopy, &Font_7x10, 1);
				SSD1306_UpdateScreen();
			}}


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 9;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00702681;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 71;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA1 LD2_Pin PA6 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|GPIO_PIN_6|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
