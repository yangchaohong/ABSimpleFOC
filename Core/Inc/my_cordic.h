#ifndef CORDIC_H
#define CORDIC_H

#include "stm32g4xx_hal.h"

extern int32_t pInBuff;         //输入数据
extern int32_t pOutBuff[2];     //输出数据

extern CORDIC_HandleTypeDef hcordic;

void cordic_init_config(void);
void cordic_calculate_start(float arg1);

void cordic_get_result(float *res1, float *res2);

#endif /* CORDIC_H */
