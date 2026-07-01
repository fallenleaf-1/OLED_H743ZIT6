#include "heart_adc.h"
#include "adc.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* --- 算法私有变量 --- */
static uint16_t rate[10];                
static uint32_t sampleCounter = 0;       // 改为纯定时器步进计数
static uint32_t lastBeatTime = 0;        
static uint16_t P = 2048;                
static uint16_t T = 2048;                
static uint16_t thresh = 2048;           
static uint16_t amp = 100;               
static bool firstBeat = true;            
static bool secondBeat = false;          
static bool Pulse = false;               

/* --- 外部公开变量 --- */
uint16_t IBI = 600;                      
uint16_t BPM = 0;                        
bool QS = false;                         

/* --- 极速 ADC 采样与滤波 (专为中断设计) --- */
static uint16_t GET_ADC_FAST(void)
{
    static uint32_t filter_val = 0;
    uint16_t raw_val = 0;
    
    // 1. 单次快速采样 (耗时极短)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
        raw_val = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // 2. 软件低通滤波 (替代之前的冒泡排序，效果相同但速度快上百倍)
    // 刚开机时初始化滤波器
    if (filter_val == 0) filter_val = raw_val; 
    
    // 指数移动平均滤波: 新值占1/8，旧值占7/8
    filter_val = (filter_val * 7 + raw_val) / 8; 
    
    return (uint16_t)filter_val;
}

/**
  * @brief 整合后的心率处理函数 (由 TIM3 2ms 中断调用)
  */
void HeartRate_Process(void)
{
    uint16_t Signal = GET_ADC_FAST();    // 1. 获取平滑后的ADC值
    
    // 2. 绝对精准的时间步进 (每次中断固定加 2ms)
    sampleCounter += 2;                  
    uint32_t Num = sampleCounter - lastBeatTime; 

    // --- 寻找信号的波谷 ---
    if (Signal < thresh && Num > (IBI / 5) * 3) {
        if (Signal < T) T = Signal;
    }

    // --- 寻找信号的波峰 ---
    if (Signal > thresh && Signal > P) {
        P = Signal;
    }

    // --- 核心检测逻辑 ---
    if (Num > 250) { // 不应期保护
        if ((Signal > thresh) && (Pulse == false) && (Num > (IBI / 5) * 3)) {
            Pulse = true;                       
            IBI = sampleCounter - lastBeatTime; 
            lastBeatTime = sampleCounter;       

            if (firstBeat) {                    
                firstBeat = false;
                return;
            }
            if (secondBeat) {                   
                secondBeat = false;
                for (int i = 0; i < 10; i++) rate[i] = IBI; 
            }

            uint32_t runningTotal = 0;
            for (int i = 0; i < 9; i++) {
                rate[i] = rate[i + 1];
                runningTotal += rate[i];
            }
            rate[9] = IBI;
            runningTotal += rate[9];

            BPM = 60000 / (runningTotal / 10); 
            QS = true;                         
        }
    }

    // --- 信号回落，更新动态阈值 ---
    if (Signal < thresh && Pulse == true) {
        Pulse = false;
        amp = P - T;               
        thresh = amp * 0.6 + T;      
        P = thresh;                
        T = thresh;                
    }

    // --- 超时保护：2.5秒没检测到心跳，重置参数 ---
    if (Num > 2500) {
        // 关键修复：不要硬编码2048！让阈值自动跟随当前传感器的静态电压
        thresh = Signal; 
        P = Signal;
        T = Signal;
        lastBeatTime = sampleCounter;
        firstBeat = true;
        secondBeat = false;
        BPM = 0;
    }
}

void HeartRate_Send_Message(void)
{
    char msg[64];
    sprintf(msg, "BPM = %d, IBI = %d\r\n", BPM, IBI);
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
}