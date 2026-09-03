#ifndef __MT6816_H
#define __MT6816_H

#include "stm32g4xx_hal.h"

/* 绑定外部 SPI1 句柄 */
extern SPI_HandleTypeDef hspi1;

/* MT6816 SPI 读写操作码定义 */
#define MT6816_READ_OP 0x80
#define MT6816_WRITE_OP 0x00

/* 寄存器映射 */
#define MT6816_REG_ANGLE_H 0x03
#define MT6816_REG_ANGLE_L 0x04

/* 状态位掩码 */
#define MT6816_NO_MAG_WARN_BIT 0x02

#define MT6816_SPI_CS_H() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1)
#define MT6816_SPI_CS_L() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0)

/* 数据结构体 */
typedef struct {
  uint16_t combined_data;
  uint16_t raw_data;
  uint16_t angle_raw;
  float angle_rad;
  uint8_t no_magnet;
  uint8_t parity_error;
} MT6816_Info_t;

/* 函数声明 */
void MT6816_Init(void);
uint8_t MT6816_ReadAngle(MT6816_Info_t *encoder);

#endif /* __MT6816_H */