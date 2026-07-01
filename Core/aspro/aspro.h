#ifndef __ASPRO_H
#define __ASPRO_H
#include "usart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

typedef struct 
{
    const char *label;
    char code;
} LabelMap;
int Process_All_Data(char *input, int bpm, float fall_prob, char *output);
#endif