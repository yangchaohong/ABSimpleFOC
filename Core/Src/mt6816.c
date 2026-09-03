#include "mt6816.h"

/**
 * @brief  MT6816 硬件片选版初始化
 */
void MT6816_Init(void) {
  /* 确保 SPI1 正常使能 */
  __HAL_SPI_ENABLE(&hspi1);
  MT6816_SPI_CS_H();
}

/**
 * @brief  16位偶校验计算
 */
static uint8_t MT6816_ParityCheck(uint16_t data) {
  data ^= data >> 8;
  data ^= data >> 4;
  data ^= data >> 2;
  data ^= data >> 1;
  return (data & 1);
}

/**
 * @brief  通过硬件SPI1连续单次传输读取MT6816绝对角度值 (原子锁存，彻底消除跨字节高低位时序跳变)
 */
uint8_t MT6816_ReadAngle(MT6816_Info_t *encoder) {
  uint16_t tx_buf[2];
  uint16_t rx_buf[2] = {0};

  tx_buf[0] = (MT6816_READ_OP | MT6816_REG_ANGLE_H) << 8; // 0x8300
  tx_buf[1] = (MT6816_READ_OP | MT6816_REG_ANGLE_L) << 8; // 0x8400

  // 1. 拉低片选 (MT6816 在 CS 下降沿锁存当前 14-bit 绝对角度)
  MT6816_SPI_CS_L();

  // 2. 连续传输 2 个 16-bit 字 (Size=2 对应 16-bit 模式下的 2 个数据字)
  if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_buf, (uint8_t *)rx_buf, 2, 5) != HAL_OK) {
    MT6816_SPI_CS_H();
    return 0;
  }

  // 3. 释放片选
  MT6816_SPI_CS_H();

  /* 4. 数据解算：提取同一时刻锁存的高 8 位与低 8 位 */
  uint16_t combined = ((rx_buf[0] & 0x00FF) << 8) | (rx_buf[1] & 0x00FF);
  encoder->combined_data = combined;
  encoder->raw_data = combined;

  // 5. 奇偶校验
  if (MT6816_ParityCheck(combined) != 0) {
    encoder->parity_error = 1;
    return 0; // 校验失败，保留上一帧有效角度
  } else {
    encoder->parity_error = 0;
  }

  // 6. 磁铁警告检测
  if (combined & MT6816_NO_MAG_WARN_BIT) {
    encoder->no_magnet = 1;
    return 0;
  } else {
    encoder->no_magnet = 0;
  }

  // 7. 解析 14-bit 角度 (0 ~ 16383)
  encoder->angle_raw = combined >> 2;
  encoder->angle_rad = (float)encoder->angle_raw * (2.0f / 16384.0f);

  encoder->angle_rad = 2.0f - encoder->angle_rad;

  return 1;
}