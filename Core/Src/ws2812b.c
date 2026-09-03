#include "ws2812b.h"

uint16_t RGB_BUFF[BUFFER_SIZE];
RGB_Color_t LED_Data[LED_NUM];

/**
  * @brief  初始化/复位缓冲区
  * @param  None
  * @retval None
  */
void WS2812_Init(void)
{
    int i;
    // 将所有灯珠数据部分初始化为逻辑 0 的占空比
    for (i = 0; i < LED_NUM * ONE_LED_BITS; i++) {
        RGB_BUFF[i] = WS2812_0;
    }
    // 尾部留出复位信号，全给 0 占空比（在传输结束后保持低电平）
    for (i = LED_NUM * ONE_LED_BITS; i < BUFFER_SIZE; i++) {
        RGB_BUFF[i] = 0;
    }
}

/**
  * @brief  设置单个灯珠的颜色
  * @param  LedId: 灯珠编号 (0 到 LED_NUM-1)
  * @param  Color: RGB 颜色结构体
  */
void WS2812_Set_LEDColor(uint16_t LedId, RGB_Color_t Color)
{
    if (LedId >= LED_NUM) return;

    // 缓存应用层颜色
    LED_Data[LedId] = Color;

    // 填充到 DMA 缓冲区。WS2812B 接收顺序是：G -> R -> B，高位在前 (MSB)
    uint16_t buff_idx = LedId * ONE_LED_BITS;

    // 填充绿光 (G) 8位
    for (int i = 7; i >= 0; i--) {
        RGB_BUFF[buff_idx++] = ((Color.G >> i) & 0x01) ? WS2812_1 : WS2812_0;
    }
    // 填充红光 (R) 8位
    for (int i = 7; i >= 0; i--) {
        RGB_BUFF[buff_idx++] = ((Color.R >> i) & 0x01) ? WS2812_1 : WS2812_0;
    }
    // 填充蓝光 (B) 8位
    for (int i = 7; i >= 0; i--) {
        RGB_BUFF[buff_idx++] = ((Color.B >> i) & 0x01) ? WS2812_1 : WS2812_0;
    }
}

/**
  * @brief  将缓冲区数据通过 DMA+PWM 发送出去（针对 TIM8_CH4N 互补通道）
  */
void WS2812_Update(void)
{
    HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, (uint32_t *)RGB_BUFF, BUFFER_SIZE);
}

/**
  * @brief  DMA 传输完成回调函数
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8) {
        // 传输完成后立即停止互补通道的 DMA
        HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_3);
    }
}