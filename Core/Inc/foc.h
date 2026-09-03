#ifndef FOC_H
#define FOC_H

#define SQRT_3P0 1.732050f
#define PI 3.14159265358979323846f

#define MAX_V_BUS_LIMIT 8.0f // 最大相电压限制 (V)

// PID 控制器状态与参数
struct PID_Controller {
  float Kp, Ki, Kd;
  float integral;
  float prev_error;
  float out_limit;      // 输出限幅
  float integral_limit; // 积分抗饱和限幅
};
// 电机状态与参数
struct FOC_Motor {
  // 物理参数
  int pole_pairs;    // 极对数 P
  float zero_offset; // 电角度初始偏移角
  float v_bus;       // 母线电压 (Vdc)
  int pwm_period;    // 定时器 ARR 计数值
  signed char direction; // 旋转方向
  // 传感器测量值
  float mech_angle; // 机械角度 theta_m (rad)
  float elec_angle; // 电角度 theta_e (rad)
  float velocity;   // 实际角速度 (rad/s)
  float Ia, Ib, Ic; // 三相相电流 (A)
  // 控制环路目标值
  float target_angle;    // 目标位置 (rad)
  float target_velocity; // 目标速度 (rad/s)
  float target_Id;       // 直轴目标电流 (通常设为 0，弱磁时为负)
  float target_Iq;       // 交轴目标电流 (直接对应输出力矩)
  // 中间变换变量
  float I_alpha, I_beta; // Clarke 变换输出
  float Id, Iq;          // Park 变换输出 (实际反馈电流)
  float Vd, Vq;          // 电流环 PI 计算输出电压
  float V_alpha, V_beta; // 逆 Park 变换输出电压
  float Ta, Tb, Tc;      // SVPWM 计算出的三相 PWM 占空比 / 计数值
};

void Clarke_Transform(float Ia, float Ib, float Ic, float *I_alpha,
                      float *I_beta);

void Park_Transform(float I_alpha, float I_beta, float theta_e, float *Id,
                    float *Iq);

void Inv_Park_Transform(float Vd, float Vq, float theta_e, float *V_alpha,
                        float *V_beta);

void SVPWM_Calculate(float V_alpha, float V_beta, float Vdc, int ARR, float *Ta,
                     float *Tb, float *Tc);

float PID_Compute(struct PID_Controller *pid, float error);

void FOC_Run(struct FOC_Motor *motor, struct PID_Controller *pid_id,
             struct PID_Controller *pid_iq);

void OPENLOOP_Run(struct FOC_Motor *motor, float Vd, float theta_virtual);

float Normalize_Angle_0_to_2PI(float angle);
#endif // FOC_H