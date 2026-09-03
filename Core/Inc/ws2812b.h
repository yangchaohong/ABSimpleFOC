#ifndef __WS2812B_H
#define __WS2812B_H
#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim8;

/* --- WS2812B 参数定义 --- */
#define LED_NUM             1       // 灯珠数量
#define ONE_LED_BITS        24      // 每个灯珠 24 位数据 (G-R-B)
#define RESET_BITS          40      // 复位信号所需的低电平点数 (大于280us)

#define WS2812_0            45      
#define WS2812_1            115      

/* DMA 发送缓冲区大小 = (灯珠数 * 24位) + 复位低电平数据 */
#define BUFFER_SIZE         ((LED_NUM * ONE_LED_BITS) + RESET_BITS)

extern uint16_t RGB_BUFF[BUFFER_SIZE];    // DMA 传输的数据缓冲区

/* RGB 颜色结构体 */
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_t;

extern RGB_Color_t LED_Data[LED_NUM];      // 存储应用层的各个灯珠颜色

void WS2812_Init(void);
void WS2812_Set_LEDColor(uint16_t LedId, RGB_Color_t Color);
void WS2812_Update(void);

#endif /* __WS2812B_H */