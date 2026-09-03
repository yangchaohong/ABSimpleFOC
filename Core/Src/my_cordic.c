#include "my_cordic.h"

int32_t pInBuff;            //输入数据
int32_t pOutBuff[2];        //输出数据

/**
 * @brief  初始化 CORDIC 配置：SINE 模式，双输出
 * @note   在 MX_CORDIC_Init() 之后调用
 */
void cordic_init_config(void)
{
    CORDIC_ConfigTypeDef cordic_config;
    cordic_config.Function  = CORDIC_FUNCTION_SINE;
    cordic_config.Scale     = CORDIC_SCALE_0;
    cordic_config.InSize    = CORDIC_INSIZE_32BITS;
    cordic_config.OutSize   = CORDIC_OUTSIZE_32BITS;
    cordic_config.NbWrite   = CORDIC_NBWRITE_1;
    cordic_config.NbRead    = CORDIC_NBREAD_2;    /* 两次读：sin, cos */
    cordic_config.Precision = CORDIC_PRECISION_6CYCLES;
    HAL_CORDIC_Configure(&hcordic, &cordic_config);
}

// CORDIC 计算函数（直接寄存器操作，确保等待 RRDY）
void cordic_calculate_start(float arg1)
{
    // 1. 修复防溢出映射：将 [0, 2] 映射到 [-1, 1]
    if(arg1 > 1.0f) {
        arg1 -= 2.0f;
    }

    // 2. 修复 Q31 溢出：限制最大值为 0.99999f，防止变成负数
    if(arg1 >= 1.0f)  arg1 = 0.99999f;
    if(arg1 <= -1.0f) arg1 = -1.0f;

    pInBuff = (int32_t)(arg1 * 2147483647.0f); // 乘以 2^31 - 1

    // 【核心修复】直接寄存器操作，确保等待计算完成
    // 标准 HAL_CORDIC_Calculate 在 NbCalc=1 时不等待 RRDY，导致读出垃圾
    WRITE_REG(CORDIC->WDATA, (uint32_t)pInBuff);

    // 等待结果就绪（CORDIC 6 个周期 @ 128MHz ≈ 47ns）
    while (READ_BIT(CORDIC->CSR, CORDIC_CSR_RRDY) == 0);

    // 读结果（NBREAD=2：第一次读 sin，第二次读 cos）
    pOutBuff[0] = (int32_t)READ_REG(CORDIC->RDATA); // sin
    pOutBuff[1] = (int32_t)READ_REG(CORDIC->RDATA); // cos
}

void cordic31_to_value(int cordic31, float *res)
{
    // 2147483648.0f 即 2^31。直接除以它，有符号整型会自动转换为 -1.0f 到 +1.0f
    *res = (float)cordic31 / 2147483648.0f;
}

void cordic_get_result(float *res1, float *res2)
{
    cordic31_to_value(pOutBuff[0], res1);  // 正弦值
    cordic31_to_value(pOutBuff[1], res2);  // 余弦值
}
