#include "aspro.h"
/* 定义映射结构 */


LabelMap map[] = {
    {"person", 'A'},
    {"rider", 'B'},
    {"car", 'C'},
    {"truck", 'D'},
    {"bus", 'E'},
    {"motorcycle", 'F'},
    {"bicycle", 'G'}
};

void Process_All_Data(char *input, int bpm, float fall_prob, char *output) {
    char label[20] = "unknown";
    float distance = 0.0f;
    char category_code = 'Z'; // 默认为 Z 表示未知物体

    // 1. 解析输入的视觉字符串 "{label}:{distance}"
    if (sscanf(input, "%[^:]:%f", label, &distance) >= 1) {
        // 2. 查找匹配的代码 (A-G)
        for (int i = 0; i < 7; i++) {
            if (strcmp(label, v_map[i].label) == 0) {
                category_code = v_map[i].code;
                break;
            }
        }
    }

    // 3. 将所有数据打包成一个字符串帧
    // 格式：BPM:75,FALL:0.10,OBJ:A,DIST:1.25\r\n
    // 你可以根据上位机的要求调整这个格式
    sprintf(output, "BPM:%d,FALL:%.2f,OBJ:%c,DIST:%.2f\r\n", 
            bpm, fall_prob, category_code, distance);
}