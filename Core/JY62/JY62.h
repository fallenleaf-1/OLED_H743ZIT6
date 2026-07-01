#ifndef __JY62_H
#define __JY62_H

#include "stm32h7xx_hal.h"
#include "filter.h"

// 配置参数
#define JY_BUF_SIZE      128    // 每个实例独立的环形缓冲区大小
#define JY_FRAME_LEN     11     // 标准协议长度
#define G_CONST          9.80665f

/* --------------------- 数据结构 --------------------- */

typedef struct {
    float q0, q1, q2, q3;
} JY62_Quat_t;

typedef struct {
    float Roll, Pitch, Yaw;
} JY62_Euler_t;

typedef struct {
    // 原始及剥离重力后的数据
    float acc[3];       // 0:x, 1:y, 2:z 
    float av[3];        // 角速度
    float angle[3];     // 欧拉角

    // 滤波后的最终输出
    float acc_fil[3];
    float av_fil[3];
    float angle_fil[3];

    JY62_Euler_t Euler;
    JY62_Quat_t  Quat;

    // 内部缓冲区与状态机
    uint8_t  rx_fifo[JY_BUF_SIZE]; // 独立缓冲区
    uint16_t head;
    uint16_t tail;

    uint8_t  msg_buf[JY_FRAME_LEN];
    uint8_t  st_index;
    uint8_t  state;

    // 滤波器实例
    KalmanFilter  kf_acc[3];
    pt2Filter_t   pt2_av[3];
    PT1Filter_t   pt1_angle[3];

    // 突变抑制辅助
    float last_raw_acc[3];
		
		float acc_init[3];    // 记录初始加速度
    float angle_init[3];  // 记录初始欧拉角
    uint8_t is_initialized; // 是否已捕获第一帧的标志位（0:未捕获, 1:已捕获）
		
		//计算摔倒概率值
		float fall_probability;
} JY62_Instance_t;

/* --------------------- 函数接口 --------------------- */

void JY62_Init(JY62_Instance_t *dev);
// 核心输出函数
void JY62_Decode_Buffer(JY62_Instance_t *dev, uint16_t len);
// JSON 输出
//void JY62_Send_Json(UART_HandleTypeDef *huart, JY62_Instance_t *dev);
//摔倒概率估算
float Fall_Probability(JY62_Instance_t *dev);
#endif