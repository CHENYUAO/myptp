#include <math.h>
#include "tmv.h"
#include "print.h"
// parameters
float kf_T = 1e-3;
float kf_Q = 1e-5;
float kf_R = 1e-5;
float kf_x_prev = 0;
int64_t kf_counter = 0;
float kf_P = 0.0;
float kf_r = 0.0;
float kf_q = 0.0;
float kf_RR = 0.0;
const float kf_b = 0.99;
float kf_mean_prev = 0.0;
float kf_var_prev = 0.0;

// 滤波程序
int64_t KalmanFilter(int64_t raw_offset){
    float x_predict =  kf_x_prev;
    kf_P = kf_P + kf_Q;

    float kf_K = kf_P / (kf_P + kf_R);
    float x_result = (int64_t)(x_predict + (kf_K * ((float)raw_offset - (float)x_predict)));

    kf_P = (1.0 - kf_K) * kf_P;
    kf_x_prev = x_result;
    return (int64_t)x_result;
}

int64_t KalmanFilterPro(int64_t raw_offset, int64_t raw_delay){
    kf_counter++;
    float x_predict = kf_x_prev + kf_q;
    kf_P = kf_P + kf_Q;
    float kf_e = (float)raw_offset - x_predict - kf_r;

    float kf_K = kf_P /(kf_P + kf_RR);
    float x_result = x_predict + kf_K * ((float)raw_offset - x_predict);
    kf_P = (1.0 - kf_K) * kf_P;

    float dk = (1.0 - kf_b)/(1.0 - pow(kf_b,kf_counter));
    kf_r = (1.0 - dk) * kf_r + dk * ((float)raw_offset - x_predict);
    kf_q = (1.0 - dk) * kf_q + dk * (x_result - kf_x_prev);
    kf_R = (1.0 - dk) * kf_R + dk * (kf_e * kf_e - kf_P);
    kf_Q = (1.0 - dk) * kf_Q + dk * (kf_K * kf_K * kf_e * kf_e);
    float n = (float)kf_counter;
    // 方差和平均值
    float kf_mean = ((float)kf_counter * kf_mean_prev + (float)raw_delay) / (float)(kf_counter + 1);
    float kf_var = ( 
        ( n/(n + 1) ) * kf_var_prev) -
        ( n/((n + 1)*(n + 1))) * (kf_mean_prev * kf_mean_prev) + 
        ((n + 2) / ((n + 1)*(n + 1)) * (float)raw_delay);
    kf_RR = kf_R + kf_var;

    kf_mean_prev = kf_mean;
    kf_var_prev = kf_var;
    pr_info( "before filter:%lld\tafter filter:%lld", raw_offset, (int64_t)x_result);
    return (int64_t)x_result;
}

