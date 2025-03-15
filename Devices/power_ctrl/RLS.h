#ifndef FRLS_H
#define FRLS_H

#include <math.h>

#define K3_CONST 3.8

// 定义 FRLS_Estimator 结构体，保存带遗忘因子递归最小二乘（FRLS）算法的状态 
typedef struct {
    double theta[2];         // 参数估计：theta[0] 表示 k1，theta[1] 表示 k2 
    double P[2][2];          // 协方差矩阵 
    double forget_factor;    // 遗忘因子，取值范围 (0, 1] 
} FRLS_Estimator;

/* 
 * 初始化 FRLS_Estimator 实例 
 * 参数： 
 *   est           - 指向 FRLS_Estimator 实例的指针 
 *   forget_factor - 遗忘因子（例如 0.98） 
 *   delta         - 用于初始化协方差矩阵 P 主对角线的放大因子（例如 1000） 
 */ 
void frls_init(FRLS_Estimator* est, double forget_factor, double delta);

/*
 * 使用一次累计观察数据更新 FRLS 参数估计 
 * 参数： 
 *   est     - 指向 FRLS_Estimator 的指针 
 *   S_abs   - 累计 |omega|： Σ|ω_i| 
 *   S_tau2  - 累计 τ_i2： Σ(τ_i)^2 
 *   S_base  - 累计 (τ_i * ω_i)： Σ(τ_i * ω_i) 
 *   P_meas  - 累计测量值，由模型计算： 
 *             P = S_base + k1_true * S_abs + k2_true * S_tau2 + 103 
 */
void frls_update(FRLS_Estimator* est, double S_abs, double S_tau2, double S_base, double P_meas);

#endif /* FRLS_H */