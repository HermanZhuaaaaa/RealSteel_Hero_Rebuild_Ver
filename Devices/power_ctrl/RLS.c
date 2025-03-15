#include "RLS.h"
#include <stdlib.h>
#include <stdio.h>



// 初始化 FRLS_Estimator 实例
void frls_init(FRLS_Estimator* est, double forget_factor, double delta) {
    est->forget_factor = forget_factor;
    // 初始时将参数估计设为 0 
    est->theta[0] = 0.0;
    est->theta[1] = 0.0;
    // 初始化协方差矩阵 P 为 delta 倍的单位矩阵 
    est->P[0][0] = delta;   est->P[0][1] = 0.0;
    est->P[1][0] = 0.0;     est->P[1][1] = delta;
}

// 使用累计观察数据更新 FRLS 参数估计 
void frls_update(FRLS_Estimator* est, double S_abs, double S_tau2, double S_base, double P_meas) {
    /*
       模型为：
         P = S_base + k1 * S_abs + k2 * S_tau2 + 103 
       整理为： 
         y = P - (S_base + 103) = k1 * S_abs + k2 * S_tau2 
       构造回归向量 φ = [S_abs, S_tau2] 
    */
    double y = P_meas - (S_base + K3_CONST);
    double phi[2] = { S_abs, S_tau2 };

    // 计算增益向量： k_gain = P * φ / (forget_factor + φ? * P * φ) 
    double Pphi[2];
    Pphi[0] = est->P[0][0] * phi[0] + est->P[0][1] * phi[1];
    Pphi[1] = est->P[1][0] * phi[0] + est->P[1][1] * phi[1];

    double phiTPphi = phi[0] * Pphi[0] + phi[1] * Pphi[1];
    double denom = est->forget_factor + phiTPphi;

    double k_gain[2];
    k_gain[0] = Pphi[0] / denom;
    k_gain[1] = Pphi[1] / denom;

    // 计算预测值： yhat = φ? * theta，并计算误差： error = y - yhat 
    double yhat = phi[0] * est->theta[0] + phi[1] * est->theta[1];
    double error = y - yhat;

    // 更新参数估计 
    est->theta[0] += k_gain[0] * error;
    est->theta[1] += k_gain[1] * error;

    // 更新协方差矩阵 P，公式： P = (I - k * φ?) * P / forget_factor 
    double newP[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            newP[i][j] = est->P[i][j] - k_gain[i] * (est->P[0][j] * phi[0] + est->P[1][j] * phi[1]);
            newP[i][j] /= est->forget_factor;
        }
    }
    est->P[0][0] = newP[0][0];
    est->P[0][1] = newP[0][1];
    est->P[1][0] = newP[1][0];
    est->P[1][1] = newP[1][1];
}