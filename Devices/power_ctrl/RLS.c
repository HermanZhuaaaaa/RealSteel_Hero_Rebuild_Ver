#include "RLS.h"
#include <stdlib.h>
#include <stdio.h>



// ��ʼ�� FRLS_Estimator ʵ��
void frls_init(FRLS_Estimator* est, double forget_factor, double delta) {
    est->forget_factor = forget_factor;
    // ��ʼʱ������������Ϊ 0 
    est->theta[0] = 0.0;
    est->theta[1] = 0.0;
    // ��ʼ��Э������� P Ϊ delta ���ĵ�λ���� 
    est->P[0][0] = delta;   est->P[0][1] = 0.0;
    est->P[1][0] = 0.0;     est->P[1][1] = delta;
}

// ʹ���ۼƹ۲����ݸ��� FRLS �������� 
void frls_update(FRLS_Estimator* est, double S_abs, double S_tau2, double S_base, double P_meas) {
    /*
       ģ��Ϊ��
         P = S_base + k1 * S_abs + k2 * S_tau2 + 103 
       ����Ϊ�� 
         y = P - (S_base + 103) = k1 * S_abs + k2 * S_tau2 
       ����ع����� �� = [S_abs, S_tau2] 
    */
    double y = P_meas - (S_base + K3_CONST);
    double phi[2] = { S_abs, S_tau2 };

    // �������������� k_gain = P * �� / (forget_factor + ��? * P * ��) 
    double Pphi[2];
    Pphi[0] = est->P[0][0] * phi[0] + est->P[0][1] * phi[1];
    Pphi[1] = est->P[1][0] * phi[0] + est->P[1][1] * phi[1];

    double phiTPphi = phi[0] * Pphi[0] + phi[1] * Pphi[1];
    double denom = est->forget_factor + phiTPphi;

    double k_gain[2];
    k_gain[0] = Pphi[0] / denom;
    k_gain[1] = Pphi[1] / denom;

    // ����Ԥ��ֵ�� yhat = ��? * theta���������� error = y - yhat 
    double yhat = phi[0] * est->theta[0] + phi[1] * est->theta[1];
    double error = y - yhat;

    // ���²������� 
    est->theta[0] += k_gain[0] * error;
    est->theta[1] += k_gain[1] * error;

    // ����Э������� P����ʽ�� P = (I - k * ��?) * P / forget_factor 
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