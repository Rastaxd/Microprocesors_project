/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "frame.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <frame.h>  // Obsługa ramek
#include <stdlib.h> // Dla atoi
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUFFER_SIZE 128 // Rozmiar bufora danych z czujnika
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define USART_TXBUF_LEN 3072 // Bufor nadawczy
#define USART_RXBUF_LEN 1024 // Bufor odbiorczy
uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];

// Wskaźniki Nadawcze
__IO int USART_TX_Empty=0;
__IO int USART_TX_Busy=0;

// Wskaźniki idbiorcze
__IO int USART_RX_Empty=0;
__IO int USART_RX_Busy=0;

// Bufor danych odebranych przez czujnik
uint8_t sensorBuffer[SENSOR_BUFFER_SIZE];
volatile uint16_t sensorIndex = 0;
volatile uint8_t liveModeEnabled = 0; // Flaga trybu LIVEG

// Dodaj zmienne globalne
static FrameState frameState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void processCommand(Frame *frame);
void sendResponse(const char *message);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Weryfikacja, czy w buforze są nowe dane
uint8_t USART_kbhit(){
	if(USART_RX_Empty==USART_RX_Busy){
		return 0;
	}else{
		return 1;
	}
}

// Funkcja pobiera jeden znak z bufora odbiorczego
int16_t USART_getchar(){
int16_t tmp;
	if(USART_RX_Empty!=USART_RX_Busy){
		 tmp=USART_RxBuf[USART_RX_Busy];// Pobierz dane
		 USART_RX_Busy++;
		 if(USART_RX_Busy >= USART_RXBUF_LEN)USART_RX_Busy=0; // Zawijanie wskaźnika
		 return tmp; // Zwróc odebrany bajt
	}else return -1; // Brak danych
}



void USART_fsend(char* format,...){
char tmp_rs[128];
int i;
__IO int idx;
va_list arglist;

  va_start(arglist,format);
  vsprintf(tmp_rs,format,arglist); // Formatowanie danych do bufora
  va_end(arglist);

  idx=USART_TX_Empty;

  for(i=0;i<strlen(tmp_rs);i++){
	  USART_TxBuf[idx]=tmp_rs[i]; // Dane do bufora nadawczego
	  idx++;
	  if(idx >= USART_TXBUF_LEN)idx=0; // Zawijanie wskaznika
  }
  __disable_irq(); // Wylaczenie przerwan
  if((USART_TX_Empty==USART_TX_Busy)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){//sprawdzic dodatkowo zajetosc bufora nadajnika
	  // Rozpocznij przesylanie jesli bufor pusty i transmisja wolna
	  USART_TX_Empty=idx;
	  uint8_t tmp = USART_TxBuf[USART_TX_Busy];
	  USART_TX_Busy++;
	  if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
	  HAL_UART_Transmit_IT(&huart2, &tmp, 1); // Wysylanie z przerwaniem
  }else{
	  USART_TX_Empty=idx; // Aktualizacja wskaznika
  }
  __enable_irq(); // Włączenie przerwań
}//fsend

// Callback na Wyslanie
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart==&huart2){
    if(USART_TX_Empty!=USART_TX_Busy){
      uint8_t tmp=USART_TxBuf[USART_TX_Busy];
      USART_TX_Busy++;
      if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
      HAL_UART_Transmit_IT(&huart2, &tmp, 1);
    }
  }
}

void sendResponse(const char *message) {
  if (!message || strlen(message) > MAX_DATA_LEN) {  // Dodanie sprawdzenia NULL
      return;
  }
  Frame responseFrame;
  // Inicjalizacja ramki odpowiedzi
  responseFrame.sourceAddress[0] = 'S';
  responseFrame.sourceAddress[1] = 'T';
  responseFrame.destinationAddress[0] = 'P';
  responseFrame.destinationAddress[1] = 'C';
  responseFrame.length = strlen(message);
  memcpy(responseFrame.data, message, responseFrame.length);

  // Użyj USART_fsend zamiast HAL_UART_Transmit
  uint8_t buffer[MAX_FRAME_BUFFER_SIZE];
  uint16_t frameLength = createFrame(buffer, &responseFrame);
  USART_fsend("%.*s", frameLength, buffer);
}

void processCommand(Frame *frame) {
    if (!frame) {  // Dodanie sprawdzenia NULL
        return;
    }

    // Sprawdź poprawność adresów
    if (memcmp(&frame->sourceAddress, "PC", 2) != 0 || 
        memcmp(&frame->destinationAddress, "ST", 2) != 0) {
        sendResponse("FAL");
        return;
    }

    if (frame->length < 3) {
        sendResponse("FAL");
        return;
    }

    if (memcmp(frame->data, "GET", 3) == 0) {
        uint16_t requestedCount = ascii_to_int(&frame->data[3]);
        
        if (requestedCount == 0 || requestedCount > MAX_DATA_LEN) {
            sendResponse("TMC");
            return;
        }
        if (requestedCount > sensorIndex) {
            sendResponse("TMC"); // Za dużo danych
        } else {
            // Dodać sprawdzenie przed kopiowaniem danych:
            if (requestedCount > MAX_DATA_LEN) {
                sendResponse("TMC");
                return;
            }
            Frame dataFrame;
            // Ustawiamy ST jako nadawcę odpowiedzi
            memcpy(&dataFrame.sourceAddress, "ST", 2);
            // Ustawiamy PC jako odbiorcę odpowiedzi
            memcpy(&dataFrame.destinationAddress, "PC", 2);
            dataFrame.length = requestedCount;
            memcpy(dataFrame.data, &sensorBuffer[sensorIndex - requestedCount], requestedCount);

            uint8_t buffer[512];
            uint16_t frameLength = createFrame(buffer, &dataFrame);
            HAL_UART_Transmit(&huart2, buffer, frameLength, HAL_MAX_DELAY);

            sendResponse("CPL"); // Completed
        }
    } else if (memcmp(frame->data, "LIVEG", 5) == 0) {
        liveModeEnabled = 1;
        sendResponse("CPL"); // Completed
    } else {
        sendResponse("FAL"); // Nieprawidłowa komenda
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 if(huart==&huart2){
		 USART_RX_Empty++;
		 if(USART_RX_Empty>=USART_RXBUF_LEN)USART_RX_Empty=0; // Zawijanie wskaznika
		 HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1); // Kontynuacja odbioru

	 }
}

void resetFrameState() {
    frameState.inFrame = false;
    frameState.escapeDetected = false;
    frameState.bxIndex = 0;
}

void sendStatus(uint8_t status) {
    if (status == ERR_GOOD) {
        sendResponse("CPL");
    } else {
        sendResponse("FAL");
    }
}

void processReceivedChar(uint8_t receivedChar) {    
    if (receivedChar == FRAME_START) {
        resetFrameState();
        frameState.inFrame = true;
        frameState.bx[frameState.bxIndex++] = receivedChar;
    } 
    else if (receivedChar == FRAME_END && !frameState.escapeDetected) {
        if (frameState.inFrame) {
            frameState.bx[frameState.bxIndex++] = receivedChar;
            
            Frame decodedFrame;
            if (decodeFrame(&frameState, frameState.bxIndex, &decodedFrame)) {
                processCommand(&decodedFrame);
            } else {
                sendResponse("FAL"); // Błąd dekodowania
            }
            resetFrameState();
        }
    } 
    else if (frameState.inFrame) {
            if (frameState.escapeDetected) {
                if (receivedChar == FRAME_START_STUFF) {
                    frameState.bx[frameState.bxIndex++] = FRAME_START;
                } 
                else if (receivedChar == ESCAPE_CHAR_STUFF) {
                    frameState.bx[frameState.bxIndex++] = ESCAPE_CHAR;
                } 
                else if (receivedChar == FRAME_END_STUFF) {
                    frameState.bx[frameState.bxIndex++] = FRAME_END;
                } 
                else {
                    sendResponse("FAL"); // Błędna sekwencja escape
                    resetFrameState();
                }
                frameState.escapeDetected = false;
            } 
            else if (receivedChar == ESCAPE_CHAR) {
                frameState.escapeDetected = true;
            } 
            else {
                if (frameState.bxIndex < MAX_FRAME_WITHOUT_STUFFING) {
                    frameState.bx[frameState.bxIndex++] = receivedChar;
                } else {
                    sendResponse("FAL"); // Przepełnienie bufora
                    resetFrameState();
                }
            }
        } 
        else {
            sendResponse("FAL"); // Ramka za długa
            resetFrameState();
        }
    }
}

uint16_t ascii_to_int(const uint8_t* str) {
    uint16_t result = 0;
    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }
    return result;
}

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1);
 // Inicjalizacja odbioru w przerwaniach
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1) {
    // Sprawdzamy, czy w buforze są nowe dane
    if (USART_kbhit()) {

        int16_t receivedChar = USART_getchar();
        if (receivedChar != -1) {
            processReceivedChar((uint8_t)receivedChar);
        }
    }
}



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
