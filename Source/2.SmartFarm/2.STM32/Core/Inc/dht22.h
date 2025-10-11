#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* --------------------------------------------------------------------------
 * 핀 설정
 *  - 보드에 맞게 DHT22_PORT / DHT22_PIN을 정의하세요.
 *  - 기존 프로젝트에서 DHT11_PORT/DHT11_PIN만 있다면, 아래처럼 fallback 합니다.
 * -------------------------------------------------------------------------- */
#ifndef DHT22_PORT
# ifdef DHT11_PORT
#  define DHT22_PORT DHT11_PORT
# else
#  define DHT22_PORT GPIOC
# endif
#endif

#ifndef DHT22_PIN
# ifdef DHT11_PIN
#  define DHT22_PIN  DHT11_PIN
# else
#  define DHT22_PIN  GPIO_PIN_10
# endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 결과 구조체 */
typedef struct {
    int8_t   status;        // 1=OK, 음수=에러
    uint8_t  rh_hi, rh_lo;  // 원시 습도 상/하위 바이트
    uint8_t  t_hi,  t_lo;   // 원시 온도 상/하위 바이트 (MSB=부호)
    uint8_t  checksum;      // 체크섬 (상위 4바이트 합의 하위 8비트)
    float    humidity;      // %RH
    float    temperature;   // ℃
} DHT22_TypeDef;

/* 공용 유틸(이 파일에서 구현) */
uint32_t DWT_Delay_Init(void);
void     DWT_Delay_us(volatile uint32_t microseconds);
void     Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     Set_Pin_Input  (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* DHT22 API */
void         DHT22_Init(void);
DHT22_TypeDef DHT22_ReadData(void);

/* 주기 호출 헬퍼: 최소 2초 간격 준수용(선택사항) */
void DHT22_ReadPeriodic(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DHT22_H_ */
