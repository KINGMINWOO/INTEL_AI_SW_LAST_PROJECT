/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dht22.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE 256 // 명령어를 받아오는 버퍼의 최대 크기
#define ARR_CNT 6 // 명령어 인자의 최대 갯수
#define CMD_SIZE 100 // 명령어의 최대 길이
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* ============ 토양센서 변수 ============= */
float temp, humi, ph; // 온도, 습도, PH
int ec; // EC
/* ============ 1초 타이머(TIM3) 변수 ============= */
volatile int tim3Flag1Sec = 1; // 1초가 되었을 때를 알려주는 플래그 변수
volatile int tim3Flag1Min = 1; // 1분이 되었을 때를 알려주는 플래그 변수
volatile unsigned long long tim3Sec = 0; // 1초마다 하나씩 증가하는 변수(초단위 카운트)
volatile unsigned long long tim3Min = 0; // 1분마다 하나씩 증가하는 변수(초단위 카운트)
/* ============ 명령어(UART6) 변수 ============= */
char rxBuf[RX_BUF_SIZE]; // 라즈베리파이에서 명령어를 받을 버퍼
volatile uint8_t rx_ch; // 명령어를 한바이트씩 불러올 때 저장할 char 변수
volatile uint8_t rx_idx = 0; // 명령어 버퍼의 요소의 index
volatile uint8_t line_received = 0; // 명령어를 모두 받아왔을 때를 알려주는 플래그 변수
/* ============ 실내 온습도 조도, 가스 변수 ============= */
extern float air_temp, air_humi;
volatile uint16_t ADC1xConvertValue[2] = {0}; // 0: 조도, 1: 공기질
volatile int adcFlag = 0;
/* ============ 자동 동작 임계값 조절 변수 ============= */
volatile int airQuality = 50, landEC = 1000;
volatile float airTemp = 26.0, airHumi = 70.0, landHumi = 50.0, landPH = 6.5;
/* ============ 펌프 타이머 상태 변수 ============= */
volatile uint16_t g_water_ms_left = 0;     // 물 펌프 남은 시간(ms)
volatile uint16_t g_nutr_ms_left  = 0;     // 영양제 펌프 남은 시간(ms)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_LED_ON(int flag);
void MX_GPIO_LED_OFF(int flag);
void UART6_OnCommand(const char* line_in);

static inline void UART6_RxStart_IT(void)
{
	HAL_UART_Receive_IT(&huart6, (uint8_t*)&rx_ch, 1);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ======================================= 디버깅용 print ===========================================*/
int __io_putchar(int ch) {
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
  return ch;
}

/* ======================================= 모듈 ON/OFF ===========================================*/
void MX_GPIO_LED_ON(int pin)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, pin, GPIO_PIN_SET);
}
void MX_GPIO_LED_OFF(int pin)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, pin, GPIO_PIN_RESET);
}

void MX_GPIO_FAN_ON(int pin)
{
	HAL_GPIO_WritePin(FAN_GPIO_Port, pin, GPIO_PIN_SET);
}
void MX_GPIO_FAN_OFF(int pin)
{
	HAL_GPIO_WritePin(FAN_GPIO_Port, pin, GPIO_PIN_RESET);
}

void MX_GPIO_AC_ON(int pin)
{
	HAL_GPIO_WritePin(AC_GPIO_Port, pin, GPIO_PIN_SET);
}
void MX_GPIO_AC_OFF(int pin)
{
	HAL_GPIO_WritePin(AC_GPIO_Port, pin, GPIO_PIN_RESET);
}

void MX_GPIO_WATER_ON(int pin)
{
	HAL_GPIO_WritePin(WATER_GPIO_Port, pin, GPIO_PIN_SET);
}
void MX_GPIO_WATER_OFF(int pin)
{
	HAL_GPIO_WritePin(WATER_GPIO_Port, pin, GPIO_PIN_RESET);
}

void MX_GPIO_NUTRIENTS_ON(int pin)
{
	HAL_GPIO_WritePin(NUTRIENTS_GPIO_Port, pin, GPIO_PIN_SET);
}
void MX_GPIO_NUTRIENTS_OFF(int pin)
{
	HAL_GPIO_WritePin(NUTRIENTS_GPIO_Port, pin, GPIO_PIN_RESET);
}

void MX_GPIO_PLANT_LED_ON(int pin)
{
	HAL_GPIO_WritePin(PLANT_LED_GPIO_Port, pin, GPIO_PIN_SET);
}
void MX_GPIO_PLANT_LED_OFF(int pin)
{
	HAL_GPIO_WritePin(PLANT_LED_GPIO_Port, pin, GPIO_PIN_RESET);
}
void MX_GPIO_HUMIDIFIER_ON(int pin)
{
	HAL_GPIO_WritePin(HUMIDIFIER_GPIO_Port, pin, GPIO_PIN_SET);
}
void MX_GPIO_HUMIDIFIER_OFF(int pin)
{
	HAL_GPIO_WritePin(HUMIDIFIER_GPIO_Port, pin, GPIO_PIN_RESET);
}
/* =======================================[토양]온도, 습도, EC, PH 값 읽어오기============================================== */
static void ReadTempHumECPH()
{
	uint8_t tx_data[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
	uint8_t rx_data[19] = {0,};

	HAL_UART_Transmit(&huart1, tx_data, 8, 100);
	HAL_UART_Receive(&huart1, rx_data, 19, 100);

	if(rx_data[0] == 1 && rx_data[1] == 3)
	{
	  humi = (float)((rx_data[3] << 8) | rx_data[4]) / 10.0;
	  temp = (float)((rx_data[5] << 8) | rx_data[6]) / 10.0;
	  ec = (int)((rx_data[7] << 8) | rx_data[8]);
	  ph = (float)((rx_data[9] << 8) | rx_data[10]) / 10.0;

	  printf("<토양센서> 온도=%.1f C  습도=%.1f %%RH EC=%d us/cm  PH=%.1f\r\n",
			  temp, humi, ec, ph);
	}
}

/* =======================================1초 카운트 타이머(TIM3)============================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//1ms 마다 호출
{
    if (htim->Instance == TIM3)
    {
        static int tim3Cnt = 0;

        // 논블로킹 펌프 타이머 처리: 1ms 단위
        if (g_water_ms_left > 0) {
            if (--g_water_ms_left == 0) {
                MX_GPIO_WATER_OFF(WATER_Pin);   // 5초 만료 → 펌프 OFF
                printf("[WATER] OFF\r\n");
            }
        }
        if (g_nutr_ms_left > 0) {
            if (--g_nutr_ms_left == 0) {
                MX_GPIO_NUTRIENTS_OFF(NUTRIENTS_Pin); // 2초 만료 → 영양제 OFF
                printf("[NUTRIENTS] OFF\r\n");
            }
        }

        // 1초 플래그 로직
        tim3Cnt++;
        if (tim3Cnt >= 1000) // 1ms * 1000 = 1초
        {
            tim3Flag1Sec = 1;
            tim3Sec++;
            tim3Cnt = 0;
        }

        if (tim3Sec >= 60) // 1분마다
        {
        	tim3Flag1Min = 1;
        	tim3Min++;
        	tim3Sec = 0;
        }
    }

    // TIM2는 ADC 트리거 용으로 쓰고 있으므로 여기서는 아무 것도 안 함.
    if (htim->Instance == TIM2)
    {
    	/* no-op */
    }
}

/* =======================================라즈베리파이에서 명령어 받아오기(UART6)============================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6)
	{
		if (rx_ch == '\n' || rx_ch == '\r')
		{
			line_received = 1;
		}
		else
		{
			if (rx_idx < RX_BUF_SIZE - 1)
			{
				rxBuf[rx_idx++] = rx_ch;
			}
		}

		UART6_RxStart_IT(); // 다음 바이트 계속 수신
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6) {
    UART6_RxStart_IT();
  }
}

/* =======================================명령어 수신 시 실행되는 함수============================================== */
void UART6_HandleLine(void)
{
    // 빠른 반환: 이벤트 없으면 끝
    if (!line_received) return;

    // ---- 크리티컬 섹션: ISR과 경합 방지용 ----
    __disable_irq();
    uint16_t n = rx_idx;
    if (n >= RX_BUF_SIZE) n = RX_BUF_SIZE - 1;

    static char line[RX_BUF_SIZE];              // 로컬(정적) 라인 버퍼
    memcpy(line, (const void*)rxBuf, n);        // 전역 -> 로컬 복사
    rx_idx = 0;                                 // 전역 인덱스 리셋
    line_received = 0;                          // 이벤트 플래그 클리어
    __enable_irq();
    // ---- 크리티컬 섹션 종료 ----

    // NUL-terminate
    line[n] = '\0';

    // 1) 디버그 출력(UART2)
    printf("%s\r\n", line);

    // 2) 콜백 훅
	UART6_OnCommand(line);

    // 3) 에코: '\n' 하나 붙여서 UART6로 돌려보내기
//    if (n < RX_BUF_SIZE - 1) {
//        line[n++] = '\n';
//    }
//    HAL_UART_Transmit(&huart6, (uint8_t*)line, n, HAL_MAX_DELAY);
}

/* =======================================명령어 처리============================================== */
/* line 끝의 \r, \n, 공백 제거 */
static void rstrip(char *s)
{
    size_t n = strlen(s);
    while (n && (s[n-1] == '\r' || s[n-1] == '\n' || s[n-1] == ' ' || s[n-1] == '\t')) {
        s[--n] = '\0';
    }
}
__attribute__((weak)) void UART6_OnCommand(const char* line_in)
{
    /* 주의: line_in은 상수 포인터일 수 있으므로 안전하게 로컬 버퍼로 복사 */
    char line[64];
    strncpy(line, line_in, sizeof(line)-1);
    line[sizeof(line)-1] = '\0';
    rstrip(line);

    // 받아온 명령어 파싱을 위한 변수
    int i = 0;
    char * pToken;
    char * pArray[ARR_CNT]={0};

    pToken = strtok(line, "[@]");
    while (pToken != NULL)
    {
    	pArray[i] = pToken;
    	if(++i >= ARR_CNT)
    		break;
    	pToken = strtok(NULL, "[@]");
    }

    if(!strcmp(pArray[0], "LED")) // 생장등 컨트롤
    {
    	if(!strcmp(pArray[1], "ON"))
    	{
    		MX_GPIO_LED_ON(LD2_Pin);   // LD2 켬
    		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 65535); // 생장등 최대
			printf("LED ON\r\n");
    	}
    	else if(!strcmp(pArray[1], "OFF"))
    	{
    		MX_GPIO_LED_OFF(LD2_Pin);   // LD2 끔
    		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0); // 생장등 끔
			printf("LED OFF\r\n");
    	}
    	else if(!strcmp(pArray[1],"LOW"))
    	{
    		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 21854); // 생장등 LOW
    		printf("LED LOW\r\n");
    	}
    	else if(!strcmp(pArray[1],"MID"))
    	{
    		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 43690); // 생장등 MID
    		printf("LED MID\r\n");
    	}
    	else if(!strcmp(pArray[1],"HIGH"))
    	{
    		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 65535); // 생장등 HIGH
    		printf("LED HIGH\r\n");
    	}
    }
    else if(!strcmp(pArray[0], "AIR")) // 실내 환경 임계 값 변경
    {
    	if(!strcmp(pArray[1], "TEMP"))
    	{
    		airTemp = atoff(pArray[2]);
    		printf("실내 온도 조정: %.1f\r\n", airTemp);
    	}
    	else if(!strcmp(pArray[1], "HUMI"))
    	{
    		airHumi = atoff(pArray[2]);
    		printf("실내 습도 조정: %.1f\r\n", airHumi);
    	}
    	else if(!strcmp(pArray[1], "QUALITY"))
    	{
    		airQuality = atoi(pArray[2]);
    		printf("실내 공기질 조정: %d\r\n", airQuality);
    	}
    	if(!strcmp(pArray[1], "ON"))
    	{
    		MX_GPIO_AC_ON(AC_Pin);   // AC 켬
    		MX_GPIO_FAN_ON(FAN_Pin);	// FAN 켬
    		MX_GPIO_HUMIDIFIER_ON(HUMIDIFIER_Pin); // HUMIDIFIER 켬
			printf("AC, FAN ON\r\n");
    	}
    	else if(!strcmp(pArray[1], "OFF"))
    	{
    		MX_GPIO_AC_OFF(AC_Pin);   // AC 끔
    		MX_GPIO_FAN_OFF(FAN_Pin);	// FAN 끔
    		MX_GPIO_HUMIDIFIER_OFF(HUMIDIFIER_Pin); // HUMIDIFIER 끔
			printf("AC, FAN OFF\r\n");
    	}

    }
    else if(!strcmp(pArray[0], "LAND")) // 토양 환경 임계 값 변경
    {
    	if(!strcmp(pArray[1], "PH"))
    	{
    		landPH = atoff(pArray[2]);
    		printf("토양 PH 조정: %.1f\r\n", landPH);
    	}
    	else if(!strcmp(pArray[1], "HUMI"))
    	{
    		landHumi = atoff(pArray[2]);
    		printf("토양 습도 조정: %.1f\r\n", landHumi);
    	}
    	else if(!strcmp(pArray[1], "EC"))
    	{
    		landEC = atoi(pArray[2]);
    		printf("토양 EC 조정: %d\r\n", landEC);
    	}
    	if(!strcmp(pArray[1], "ON"))
    	{
    		MX_GPIO_WATER_ON(WATER_Pin);   // WATER 켬
    		MX_GPIO_NUTRIENTS_ON(NUTRIENTS_Pin);	// NUTRIENTS 켬
			printf("WATER, NUTRIENTS ON\r\n");
    	}
    	else if(!strcmp(pArray[1], "OFF"))
    	{
    		MX_GPIO_WATER_OFF(WATER_Pin);   // WATER 끔
    		MX_GPIO_NUTRIENTS_OFF(NUTRIENTS_Pin);	// NUTRIENTS 끔
			printf("WATER, NUTRIENTS OFF\r\n");
    	}
    }
    else
    {
        // 알 수 없는 명령 (원하면 NAK 회신)
        // const char *nak = "ERR Unknown command\n";
        // HAL_UART_Transmit(&huart6, (uint8_t*)nak, strlen(nak), 50);
    }
}



/* =======================================아날로그 신호(조도, 가스) 받아오기============================================= */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static int channel = 0;
	if (channel == 0)
	{
		uint32_t reverse = (uint32_t)4095 - (uint32_t)HAL_ADC_GetValue(hadc);
		uint32_t cds = (reverse * 100 + 2047) / 4095;
		ADC1xConvertValue[channel] = cds;
		channel = 1;
	}
	else if (channel == 1)
	{
		uint32_t gas = ((uint32_t)HAL_ADC_GetValue(hadc) * 100 + 2047) / 4095;
		ADC1xConvertValue[channel] = gas;
		channel = 0;
		adcFlag = 1;
	}

}

/* =======================================아날로그 신호(조도, 가스) 수신 시 실행되는 함수============================================= */
void ADC_HandleLine(void)
{
	if (!adcFlag) return;

	adcFlag = 0;
	printf("조도 : %d, 가스: %d\r\n", ADC1xConvertValue[0], ADC1xConvertValue[1]);
}

/* =======================================물 주기 함수============================================= */
void WaterPump10Sec(void)
{
    // 이미 동작 중이면 재시작하지 않고 무시
    if (g_water_ms_left > 0) {
        printf("[WATER] busy (%ums left)\r\n", (unsigned)g_water_ms_left);
        return;
    }

    MX_GPIO_WATER_ON(WATER_Pin);
    g_water_ms_left = 10000;   // 10초
    printf("[WATER] ON\r\n");
}

/* =======================================영양제 주기 함수============================================= */
void NutrientsPump5Sec(void)
{
	// 이미 동작 중이면 재시작하지 않고 무시
    if (g_nutr_ms_left > 0) {
        printf("[NUTRIENTS] busy (%ums left)\r\n", (unsigned)g_nutr_ms_left);
        return;
    }

    MX_GPIO_NUTRIENTS_ON(NUTRIENTS_Pin);
    g_nutr_ms_left = 5000;    // 5초
    printf("[NUTRIENTS] ON\r\n");
}

/* =======================================1분마다 자동 임계 값과 현재 값을 비교하는 함수============================================= */
void AutomaticAction(void)
{
	/*------------------------------------실내 공기----------------------------------------*/
	if(air_temp > airTemp) // 실내 온도
	{
	  // AC 켬
	  MX_GPIO_AC_ON(AC_Pin);
	  printf("[AC] ON (임계 값: %.1f, 현재 값: %.1f)\r\n", airTemp, air_temp);
	}
	else
	{
	  // AC 끔
	  MX_GPIO_AC_OFF(AC_Pin);
	  printf("[AC] OFF (임계 값: %.1f, 현재 값: %.1f)\r\n", airTemp, air_temp);
	}

	if(air_humi < airHumi) // 실내 습도
	{
	  // 가습기 켬
		MX_GPIO_HUMIDIFIER_ON(HUMIDIFIER_Pin);
		printf("[HUMIDIFIER] ON (임계 값: %.1f, 현재 값: %.1f)\r\n", airHumi, air_humi);
	}
	else
	{
	  // 가습기 끔
		MX_GPIO_HUMIDIFIER_OFF(HUMIDIFIER_Pin);
		printf("[HUMIDIFIER] OFF (임계 값: %.1f, 현재 값: %.1f)\r\n", airHumi, air_humi);
	}

	if(ADC1xConvertValue[1] > airQuality) // 실내 공기질
	{
	  // 환기 켬
	  MX_GPIO_FAN_ON(FAN_Pin);
	  printf("[FAN] ON (임계 값: %d, 현재 값: %d)\r\n", airQuality, ADC1xConvertValue[1]);
	}
	else
	{
	  // 환기 끔
	  MX_GPIO_FAN_OFF(FAN_Pin);
	  printf("[FAN] OFF (임계 값: %d, 현재 값: %d)\r\n", airQuality, ADC1xConvertValue[1]);
	}

	/*------------------------------------토양----------------------------------------*/
	if(humi < landHumi) // 토양 습도
	{
		// 물 주기(10초)
		WaterPump10Sec();
	}

	if(ph > landPH || ec < landEC) // 토양 PH & EC
	{
		// 영양제 주기(5초)
		NutrientsPump5Sec();
	}
}

/* =======================================5분마다 현재 환경 값을 DB에 저장하기 위해 UART6로 보내는 함수============================================= */
void DB_UART6(void)
{
    uint8_t sendBuf[CMD_SIZE] = {0};     // <-- char -> uint8_t

    int n = snprintf((char*)sendBuf, sizeof(sendBuf),
                     "AIR@%.1f@%.1f@%d@%d\n",
                     air_temp, air_humi, ADC1xConvertValue[1], ADC1xConvertValue[0]); // AIR@온도@습도@공기질@조도
    if (n > 0) {
    	HAL_UART_Transmit(&huart6, sendBuf, (uint16_t)n, HAL_MAX_DELAY);
    	printf("%s\r\n", sendBuf);
    }

    n = snprintf((char*)sendBuf, sizeof(sendBuf),
                 "LAND@%.1f@%.1f@%d@%.1f\n",
                 temp, humi, ec, ph); // LAND@온도@습도@ec@ph
    if (n > 0) {
    	HAL_UART_Transmit(&huart6, sendBuf, (uint16_t)n, HAL_MAX_DELAY);
    	printf("%s\r\n", sendBuf);
    }
}

/* =======================================10초마다 현재 환경 값을 디버깅하기위해 UART6로 보내는 함수============================================= */
void Debuging_UART6(void)
{
    uint8_t sendBuf[CMD_SIZE] = {0};     // <-- char -> uint8_t

    int n = snprintf((char*)sendBuf, sizeof(sendBuf),
                     "현재 스마트팜 내부 환경 => AIR@%.1f@%.1f@%d@%d\n",
                     air_temp, air_humi, ADC1xConvertValue[1], ADC1xConvertValue[0]); // AIR@온도@습도@공기질@조도
    if (n > 0) {
    	HAL_UART_Transmit(&huart6, sendBuf, (uint16_t)n, HAL_MAX_DELAY);
    	printf("%s\r\n", sendBuf);
    }

    n = snprintf((char*)sendBuf, sizeof(sendBuf),
                 "현재 토양 환경 => LAND@%.1f@%.1f@%d@%.1f\n",
                 temp, humi, ec, ph); // LAND@온도@습도@ec@ph
    if (n > 0) {
    	HAL_UART_Transmit(&huart6, sendBuf, (uint16_t)n, HAL_MAX_DELAY);
    	printf("%s\r\n", sendBuf);
    }
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n[BOOT] Soil Sensor , temp & humi (USART1=Sensor, USART6=DHT, USART2=Debug)\r\n");

  UART6_RxStart_IT();
  DHT22_Init();

  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
	  Error_Handler();
  }

  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
	  Error_Handler();
  }

  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 라즈베리파이에서 받은 명령어 처리 함수
	  UART6_HandleLine();

	  // 실내 온습도값 읽고 출력(5초에 한번 씩)
	  DHT22_ReadPeriodic();

	  // 조도, 가스값 읽어옴
	  ADC_HandleLine();

	  //1초에 한번
	  if(tim3Flag1Sec)
	  {
		  tim3Flag1Sec = 0; // 1초 플래그 초기화

		  if(!(tim3Sec%10)) //10초에 한 번
		  {
			  // (토양)온도, 습도, EC, PH값 읽고 출력
			  ReadTempHumECPH();
			  // 현재 공기, 토양 센서 값 디버깅(UART6)
			  Debuging_UART6();
		  }
	  }

	  if(tim3Flag1Min)
	  {
		  tim3Flag1Min = 0; // 1분 플래그 초기화

		  // 자동 임계 값과 현재 값을 비교
		  AutomaticAction();

		  if (!(tim3Min % 5)) // 5분에 한 번
		  {
			  DB_UART6(); // DB에 생장 환경 데이터 전송
		  }
	  }

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HUMIDIFIER_Pin|NUTRIENTS_Pin|WATER_Pin|AC_Pin
                          |FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HUMIDIFIER_Pin NUTRIENTS_Pin WATER_Pin AC_Pin
                           FAN_Pin */
  GPIO_InitStruct.Pin = HUMIDIFIER_Pin|NUTRIENTS_Pin|WATER_Pin|AC_Pin
                          |FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT_Pin */
  GPIO_InitStruct.Pin = DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
#ifdef USE_FULL_ASSERT
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
