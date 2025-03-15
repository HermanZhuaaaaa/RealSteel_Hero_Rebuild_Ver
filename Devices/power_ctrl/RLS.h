#ifndef FRLS_H
#define FRLS_H

#include <math.h>

#define K3_CONST 3.8

// ���� FRLS_Estimator �ṹ�壬������������ӵݹ���С���ˣ�FRLS���㷨��״̬ 
typedef struct {
    double theta[2];         // �������ƣ�theta[0] ��ʾ k1��theta[1] ��ʾ k2 
    double P[2][2];          // Э������� 
    double forget_factor;    // �������ӣ�ȡֵ��Χ (0, 1] 
} FRLS_Estimator;

/* 
 * ��ʼ�� FRLS_Estimator ʵ�� 
 * ������ 
 *   est           - ָ�� FRLS_Estimator ʵ����ָ�� 
 *   forget_factor - �������ӣ����� 0.98�� 
 *   delta         - ���ڳ�ʼ��Э������� P ���Խ��ߵķŴ����ӣ����� 1000�� 
 */ 
void frls_init(FRLS_Estimator* est, double forget_factor, double delta);

/*
 * ʹ��һ���ۼƹ۲����ݸ��� FRLS �������� 
 * ������ 
 *   est     - ָ�� FRLS_Estimator ��ָ�� 
 *   S_abs   - �ۼ� |omega|�� ��|��_i| 
 *   S_tau2  - �ۼ� ��_i2�� ��(��_i)^2 
 *   S_base  - �ۼ� (��_i * ��_i)�� ��(��_i * ��_i) 
 *   P_meas  - �ۼƲ���ֵ����ģ�ͼ��㣺 
 *             P = S_base + k1_true * S_abs + k2_true * S_tau2 + 103 
 */
void frls_update(FRLS_Estimator* est, double S_abs, double S_tau2, double S_base, double P_meas);

#endif /* FRLS_H */