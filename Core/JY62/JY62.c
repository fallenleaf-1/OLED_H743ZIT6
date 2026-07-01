#include "JY62.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.1415926535f
#define DEG2RAD (PI / 180.0f)
//JY62_Instance_t jy62_1;  // 第一个
//JY62_Instance_t jy62_2;  // 第二个

/**
 * @brief 突变抑制逻辑：如果当前值与上次值偏差过大，进行限幅处理
 */
/*static float Spike_Filter(float current, float last, float threshold) {
    float diff = current - last;
    if (fabsf(diff) > threshold) {
        return last + (diff > 0 ? threshold : -threshold) * 0.2f; // 限制增长率
    }
    return current;
}*/

void JY62_Init(JY62_Instance_t *dev) {
    memset(dev, 0, sizeof(JY62_Instance_t));
		dev->fall_probability = 0;
    for(int i=0; i<3; i++) {
        // 加速度卡尔曼：Q稍微调小，R增大以抑制起伏
        KalmanFilter_Init(&dev->kf_acc[i], 0.005f, 0.8f, 0.0f, 1.0f);
        // 角速度二阶低通：截止频率设为 15Hz
        pt2FilterInit(&dev->pt2_av[i], 15.0f, 100.0f, 0.707f);
        // 角度一阶低通：10Hz
        PT1Filter_InitWithFreq(&dev->pt1_angle[i], 10.0f, 100);
    }
}


void JY62_Decode_Process(JY62_Instance_t *dev) 
{
    // 1. 校验和检查
    uint8_t checksum = 0;
    for (int i = 0; i < 10; i++) {
        checksum += dev->msg_buf[i];
    }

    if (checksum != dev->msg_buf[10]) {
        return; // 校验失败，直接丢弃该帧
    }

    // 2. 数据解析 (数据为小端模式：低字节在前，高字节在后)
    // 使用 (int16_t) 进行符号扩展，处理负数情况
    int16_t raw_data[3];
    raw_data[0] = (int16_t)((dev->msg_buf[3] << 8) | dev->msg_buf[2]);
    raw_data[1] = (int16_t)((dev->msg_buf[5] << 8) | dev->msg_buf[4]);
    raw_data[2] = (int16_t)((dev->msg_buf[7] << 8) | dev->msg_buf[6]);

    // 3. 根据包头类型分支处理
    switch (dev->msg_buf[1]) 
    {
        case 0x51: // 加速度包
            // 换算公式：a = (raw / 32768) * 16g
            for (int i = 0; i < 3; i++) {
                dev->acc[i] = (float)raw_data[i] / 32768.0f * 16.0f * G_CONST;
                // 应用你 filter.c 里的滤波逻辑
                dev->acc_fil[i] = KalmanFilter_Update(&dev->kf_acc[i], dev->acc[i]);
            }
            break;

        case 0x52: // 角速度包
            // 换算公式：w = (raw / 32768) * 2000 deg/s
            for (int i = 0; i < 3; i++) {
                dev->av[i] = (float)raw_data[i] / 32768.0f * 2000.0f;
                dev->av_fil[i] = pt2FilterApply(&dev->pt2_av[i], dev->av[i]);
            }
            break;

        case 0x53: // 角度包
            // 换算公式：angle = (raw / 32768) * 180 deg
            for (int i = 0; i < 3; i++) {
                dev->angle[i] = (float)raw_data[i] / 32768.0f * 180.0f;
                // 角度通常波动较小，可用简单低通或直接记录
                dev->angle_fil[i] = dev->angle[i]; 
            }
            break;

        default:
            break;
    }
}

// 修改后的解析函数，直接处理一段内存空间
void JY62_Decode_Buffer(JY62_Instance_t *dev, uint16_t len) 
{
    // 遍历当前 DMA 接收到的所有字节
    for (uint16_t i = 0; i < len; i++) 
    {
        uint8_t data = dev->rx_fifo[i]; // 这里的 rx_fifo 仅作为线性数组使用

        // 状态机逻辑保持不变，用于处理跨包拼接
        if (dev->state == 0 && data == 0x55) 
        {
            dev->msg_buf[0] = data;
            dev->state = 1; 
            dev->st_index = 1;
        } 
        else if (dev->state == 1) 
        {
            if (data >= 0x51 && data <= 0x53) 
            {
                dev->msg_buf[1] = data;
                dev->state = 2;
                dev->st_index = 2;
            } 
            else dev->state = 0;
        } 
        else if (dev->state == 2) 
        {
            dev->msg_buf[dev->st_index++] = data;
            if (dev->st_index == JY_FRAME_LEN) // 11字节
            {
                // 调用你原本的解析函数（包含校验和计算）
                JY62_Decode_Process(dev); 
                dev->state = 0;
            }
        }
    }
}
//计算摔跤概率
float Fall_Probability(JY62_Instance_t *dev) 
{
    // --- 第一步：捕获第一帧作为姿态基准 ---
    static uint16_t init_delay_cnt = 0;
    if (dev->is_initialized == 0) 
    {
        if (init_delay_cnt < 500) // 等待滤波器稳定（500Hz下约1秒）
        { 
            init_delay_cnt++;
            return 0.0f;
        }
        
        // 记录初始姿态（作为判断倾斜角度变化的基准）
        dev->angle_init[0] = dev->angle_fil[0]; // Roll
        dev->angle_init[1] = dev->angle_fil[1]; // Pitch
        dev->angle_init[2] = dev->angle_fil[2]; // Yaw
        
				float roll_rad_init  = dev->angle_init[0] * DEG2RAD;
				float pitch_rad_init = dev->angle_init[1] * DEG2RAD;
				
				float gx_theory_init = -sinf(pitch_rad_init);
				float gy_theory_init =  sinf(roll_rad_init) * cosf(pitch_rad_init);
				float gz_theory_init =  cosf(roll_rad_init) * cosf(pitch_rad_init);
				
				dev->acc_init[0] = dev->acc_fil[0] - gx_theory_init;
				dev->acc_init[1] = dev->acc_fil[1] - gy_theory_init;
				dev->acc_init[2] = dev->acc_fil[2] - gz_theory_init;
				
        dev->is_initialized = 1;
        return 0.0f;
    }

    // --- 第二步：利用当前角度进行重力补偿 ---
    
    // 1. 将当前角度转换为弧度
    float roll_rad  = dev->angle_fil[0] * DEG2RAD;
    float pitch_rad = dev->angle_fil[1] * DEG2RAD;

    /* 2. 计算重力向量在当前设备坐标系下的投影 (单位: g)
       公式原理：基于欧拉角旋转矩阵，计算重力 [0, 0, 1] 在当前姿态下的分量
       gx = -sin(pitch)
       gy =  sin(roll) * cos(pitch)
       gz =  cos(roll) * cos(pitch)
    */
    float gx_theory = -sinf(pitch_rad);
    float gy_theory =  sinf(roll_rad) * cosf(pitch_rad);
    float gz_theory =  cosf(roll_rad) * cosf(pitch_rad);

    // 3. 计算线性加速度 (当前测量值 - 理论重力值)
    // 这样得到的 d_ax/ay/az 理论上在静止时无论什么角度都接近 0
    float d_ax = dev->acc_fil[0] - gx_theory - dev->acc_init[0];
    float d_ay = dev->acc_fil[1] - gy_theory - dev->acc_init[1];
    float d_az = dev->acc_fil[2] - gz_theory - dev->acc_init[2];

    // 4. 计算动态合加速度 SVM_delta (单位: g)
    float svm_delta = sqrtf(d_ax*d_ax + d_ay*d_ay + d_az*d_az);

    // --- 第三步：计算姿态偏差 ---
    
    // 计算当前角度相对于初始化时刻的变化量
    float d_roll  = fabsf(dev->angle_fil[0] - dev->angle_init[0]);
    float d_pitch = fabsf(dev->angle_fil[1] - dev->angle_init[1]);
    float max_tilt_delta = fmaxf(d_roll, d_pitch);

    float prob = 0.0f;

    // --- 第四步：概率判定 ---
    
    // A. 冲击权重 (80%)
    // 补偿重力后，svm_delta > 1.2g 通常就代表有明显的剧烈动作或撞击
    if (svm_delta > 1.2f) { 
        prob += 0.8f * fminf((svm_delta - 1.2f) / 2.0f, 1.0f); 
    }

    // B. 姿态改变权重 (20%)
    // 判定逻辑：相对于穿戴时的初始姿态，发生了超过 60 度的剧烈偏转
    if (max_tilt_delta > 60.0f) { 
        prob += 0.2f * fminf((max_tilt_delta - 60.0f) / 30.0f, 1.0f);
    }

    return prob;
}

/*void JY62_Send_Json(UART_HandleTypeDef *huart, JY62_Instance_t *dev) {
    char buf[128];
    int len = sprintf(buf, "{\"key\":\"/quat\",\"value\":[%.4f,%.4f,%.4f,%.4f]}\n",
                      dev->Quat.q0, dev->Quat.q1, dev->Quat.q2, dev->Quat.q3);
    HAL_UART_Transmit(huart, (uint8_t*)buf, len, 10);
}*/