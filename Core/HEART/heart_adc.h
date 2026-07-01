#ifndef __HEART_ADC_H
#define __HEART_ADC_H

#include "main.h"
#include <stdbool.h>

// 外部可以访问的变量
extern uint16_t IBI;
extern uint16_t BPM;
extern bool QS;      // 当检测到一次有效心跳时，该标志位变为 true

// 函数声明
void HeartRate_Process(void);
void HeartRate_Send_Message(void);

#endif /* __HEART_ADC_H */