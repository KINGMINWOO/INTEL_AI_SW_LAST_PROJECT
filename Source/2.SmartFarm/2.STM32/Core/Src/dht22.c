#include "dht22.h"
#include <string.h>

float air_temp, air_humi;

/* ====================== DWT 기반 마이크로초 지연 ====================== */
uint32_t DWT_Delay_Init(void)
{
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;

  /* Disable cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  /* Enable cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

  /* Reset counter */
  DWT->CYCCNT = 0;

  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  return (DWT->CYCCNT) ? 0 : 1;
}

void DWT_Delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000U);
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds) { __NOP(); }
}

/* ====================== GPIO 방향 전환 유틸 ====================== */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;   // 외부 풀업 권장(4.7~5.1kΩ)
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/* ====================== DHT22 프로토콜 ====================== */
/* Start: MCU 0.8~1.0ms Low -> 20~30us High -> Input */
static void DHT22_Start(void)
{
  Set_Pin_Output(DHT22_PORT, DHT22_PIN);
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
  DWT_Delay_us(10);

  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
  DWT_Delay_us(1000);   // >= 0.8ms (보통 1ms)
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
  DWT_Delay_us(30);     // 20~30us
  Set_Pin_Input(DHT22_PORT, DHT22_PIN); // 센서 응답 대기
}

/* Response: ~80us Low + ~80us High */
static int8_t DHT22_Check_Response(void)
{
  int8_t rsp = -1;

  if (!HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)) {
    DWT_Delay_us(80);
    if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)) {
      rsp = 1; // OK
    } else {
      rsp = -2; // unexpected level
    }
    // 응답 High 종료까지 대기
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN));
  }
  return rsp;
}

/* 비트 읽기: High가 시작된 뒤 40us에서 샘플링
   - '0'이면 40us 시점에는 이미 Low
   - '1'이면 40us 시점에도 High */
static uint8_t DHT22_ReadByte(void)
{
  uint8_t val = 0;
  for (int j = 0; j < 8; j++) {
    // High 시작 대기
    while (!HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN));
    DWT_Delay_us(40);
    val <<= 1;
    if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)) {
      val |= 1;
    }
    // 다음 비트로 넘어가기 전, 현재 High가 끝나도록 대기
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN));
  }
  return val;
}

/* 읽기 1회: RH(2) + T(2) + CHK(1) → 변환/검증 */
DHT22_TypeDef DHT22_ReadData(void)
{
  DHT22_TypeDef d;
  memset(&d, 0, sizeof(d));

  DHT22_Start();
  d.status = DHT22_Check_Response();
  if (d.status < 0) return d;

  d.rh_hi   = DHT22_ReadByte();
  d.rh_lo   = DHT22_ReadByte();
  d.t_hi    = DHT22_ReadByte();
  d.t_lo    = DHT22_ReadByte();
  d.checksum= DHT22_ReadByte();

  /* 라인 해제(High 유지) */
  Set_Pin_Output(DHT22_PORT, DHT22_PIN);
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);

  /* 체크섬 검증 */
  uint8_t sum = (uint8_t)(d.rh_hi + d.rh_lo + d.t_hi + d.t_lo);
  if (sum != d.checksum) {
    d.status = -3; // checksum fail
    return d;
  }

  /* 값 변환 */
  uint16_t rh_raw = ((uint16_t)d.rh_hi << 8) | d.rh_lo;
  d.humidity = rh_raw / 10.0f;

  uint16_t t_raw = ((uint16_t)d.t_hi << 8) | d.t_lo;
  if (t_raw & 0x8000) { // 음수
    t_raw &= 0x7FFF;
    d.temperature = -((int)t_raw / 10.0f);
  } else {
    d.temperature = t_raw / 10.0f;
  }

  d.status = 1;
  return d;
}

/* 선택: 최소 2초 간격 주기 읽기 헬퍼 */
void DHT22_Init(void)
{
  DWT_Delay_Init();
  Set_Pin_Output(DHT22_PORT, DHT22_PIN);
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET); // idle=High
  HAL_Delay(2000); // 전원 인가 후 안정화
}

void DHT22_ReadPeriodic(void)
{
  static uint32_t last_ms = 0;
  uint32_t now = HAL_GetTick();
  if ((now - last_ms) < 5000U) return; // DHT22는 최소 2초 간격 권장
  last_ms = now;

  DHT22_TypeDef d = DHT22_ReadData();
  air_temp = d.temperature;
  air_humi = d.humidity;
  if (d.status == 1) {
     //필요 시 printf로 출력
     printf("<실내 온습도> 온도=%.1f C  습도=%.1f %%RH\r\n", d.temperature, d.humidity);
  } else {
     printf("DHT22 read fail: %d\r\n", d.status);
  }
}
