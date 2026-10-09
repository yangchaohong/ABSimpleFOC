#include "foc.h"
#include "my_cordic.h"
#include <math.h>

void Clarke_Transform(float Ia, float Ib, float Ic, float *I_alpha,
                      float *I_beta) {
  // Clarke 变换公式
  *I_alpha = Ia;
  *I_beta = (1.0f / SQRT_3P0) * (Ib - Ic);
}

void Park_Transform(float I_alpha, float I_beta, float theta_e, float *Id,
                    float *Iq) {
  // 计算 sin 和 cos
  float sin_theta = 0.0f;
  float cos_theta = 0.0f;
  cordic_calculate_start(theta_e); // 将角度归一化到 [-1, 1]
  cordic_get_result(&sin_theta, &cos_theta);

  // Park 变换公式
  *Id = I_alpha * cos_theta + I_beta * sin_theta;
  *Iq = -I_alpha * sin_theta + I_beta * cos_theta;
}

void Inv_Park_Transform(float Vd, float Vq, float theta_e, float *V_alpha,
                        float *V_beta) {
  // 计算 sin 和 cos
  float sin_theta = 0.0f;
  float cos_theta = 0.0f;
  cordic_calculate_start(theta_e); // 将角度归一化到 [-1, 1]
  cordic_get_result(&sin_theta, &cos_theta);

  // 逆 Park 变换公式
  *V_alpha = Vd * cos_theta - Vq * sin_theta;
  *V_beta = Vd * sin_theta + Vq * cos_theta;
}

void SVPWM_Calculate(float V_alpha, float V_beta, float Vdc, int ARR, float *Ta,
                     float *Tb, float *Tc) {
  float Va = V_alpha;
  float Vb = (-0.5f * V_alpha + SQRT_3P0 / 2.0f * V_beta);
  float Vc = (-0.5f * V_alpha - SQRT_3P0 / 2.0f * V_beta);

  float Vmax = fmaxf(fmaxf(Va, Vb), Vc);
  float Vmin = fminf(fminf(Va, Vb), Vc);
  float Voffset = (Vmax + Vmin) * 0.5f;

  float Va_mod = Va - Voffset;
  float Vb_mod = Vb - Voffset;
  float Vc_mod = Vc - Voffset;

  int Ta_count = (int)((Va_mod / Vdc + 0.5f) * ARR);
  int Tb_count = (int)((Vb_mod / Vdc + 0.5f) * ARR);
  int Tc_count = (int)((Vc_mod / Vdc + 0.5f) * ARR);
  int Count_top = (int)(ARR * 0.95f);
  int Count_bottom = (int)(ARR * 0.05f);

  *Ta = fminf(fmaxf(Ta_count, Count_bottom), Count_top);
  *Tb = fminf(fmaxf(Tb_count, Count_bottom), Count_top);
  *Tc = fminf(fmaxf(Tc_count, Count_bottom), Count_top);
}

float PID_Compute(struct PID_Controller *pid, float error) {
  // 积分抗饱和
  pid->integral += error * 0.0001f;
  if (pid->integral > pid->integral_limit) {
    pid->integral = pid->integral_limit;
  } else if (pid->integral < -pid->integral_limit) {
    pid->integral = -pid->integral_limit;
  }

  // PID 输出计算
  float output = pid->Kp * error + pid->Ki * pid->integral +
                 pid->Kd * (error - pid->prev_error);

  // 输出限幅
  if (output > pid->out_limit) {
    output = pid->out_limit;
  } else if (output < -pid->out_limit) {
    output = -pid->out_limit;
  }

  // 更新前一误差
  pid->prev_error = error;

  return output;
}

extern TorqueControlType torque_type;

void FOC_Run(struct FOC_Motor *motor, struct PID_Controller *pid_id,
             struct PID_Controller *pid_iq) {
  Clarke_Transform(motor->Ia, motor->Ib, motor->Ic, &motor->I_alpha,
                   &motor->I_beta);

  Park_Transform(motor->I_alpha, motor->I_beta, motor->elec_angle, &motor->Id,
                 &motor->Iq);

  if (torque_type == TORQUE_VOLTAGE) {
    // 电压模式 (开环力矩)：直接将目标设为相电压，跳过电流环 PI，防止积分饱和打满
    motor->Vd = motor->target_Id;
    motor->Vq = motor->target_Iq;
  } else {
    // 矢量电流闭环模式
    float error_Id = motor->target_Id - motor->Id;
    float error_Iq = motor->target_Iq - motor->Iq;
    motor->Vd = PID_Compute(pid_id, error_Id);
    motor->Vq = PID_Compute(pid_iq, error_Iq);
  }

  motor->Vd = fmaxf(fminf(motor->Vd, MAX_V_BUS_LIMIT), -MAX_V_BUS_LIMIT);
  motor->Vq = fmaxf(fminf(motor->Vq, MAX_V_BUS_LIMIT), -MAX_V_BUS_LIMIT);

  Inv_Park_Transform(motor->Vd, motor->Vq, motor->elec_angle, &motor->V_alpha,
                     &motor->V_beta);

  SVPWM_Calculate(motor->V_alpha, motor->V_beta, motor->v_bus,
                  motor->pwm_period, &motor->Ta, &motor->Tb, &motor->Tc);
}

void OPENLOOP_Run(struct FOC_Motor *motor, float Vd, float theta_virtual) {
  motor->Vd = fmaxf(fminf(Vd, MAX_V_BUS_LIMIT), -MAX_V_BUS_LIMIT);
  motor->Vq = 0.0f; // 开环运行时，交轴电压设为 0

  Inv_Park_Transform(motor->Vd, motor->Vq, theta_virtual, &motor->V_alpha,
                     &motor->V_beta);

  SVPWM_Calculate(motor->V_alpha, motor->V_beta, motor->v_bus,
                  motor->pwm_period, &motor->Ta, &motor->Tb, &motor->Tc);
}

float Normalize_Angle_0_to_2PI(float angle) {
  float angle_norm = fmodf(angle, 2.0f);
  if (angle_norm < 0.0f)
    angle_norm += 2.0f; // 归一化到 [0, 2.0)
  return angle_norm;
}