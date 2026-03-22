/*
 * @Author: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @Date: 2024-12-29 17:23:37
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-12-29 17:30:47
 * @FilePath: \Bsp\music.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "music.h"

#define MIN_BUZZER_PWM ((uint32_t)10000)
#define MAX_BUZZER_PWM ((uint32_t)20000)
#define MAX_PSC ((uint32_t)1000)
#define C3 131
#define C3s 139
#define D3 147
#define D3s 156
#define E3 165
#define F3 175
#define F3s 185
#define G3 196
#define G3s 208
#define A3 220
#define A3s 233
#define B3 247
#define C4 262
#define C4s 277
#define D4 294
#define D4s 311
#define E4 330
#define F4 349
#define F4s 370
#define G4 392
#define G4s 415
#define A4 440
#define A4s 466
#define B4 494
#define C5 523
#define C5s 554
#define D5 587
#define D5s 622
#define E5 659
#define F5 698
#define F5s 740
#define G5 784
#define G5s 831
#define A5 880
#define A5s 932
#define B5 988
#define C6 1047
#define zero 0

//以八分之一拍为最小单位

unsigned int notes_jiaofu[] = {
        G3,C4,D4s,D4,C4,D4s,C4,D4,C4,G3s,A3s,G3,zero,
        G3,C4,D4s,D4,C4,D4s,C4,D4,C4,G3,F3s,F3,zero,
        F3,G3s,B3,D4,  F3,G3s,B3,C4,zero,
        C4,D4s,A4s,G4s,G4,A4s,G4s,G4s,G4,G4,B3,C4,zero,
        C5,C5,B4,A4s,  A4s,D5,C5,G4s,G4,
        G4,A4s,G4,F4,  F4,G4s,G4,F4,G4
};
unsigned int duration_jiaofu[] = {
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 16, 4,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 16, 4,
        4, 4, 4, 20,    4, 4, 4, 16,4,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 16, 4,
        4, 4, 4, 12,  4, 8, 4, 4, 16,
        4, 4, 4, 12,  4, 8, 4, 4, 16
};

unsigned int notes_heimaojingzhang[] = {
        G4,zero,G4,zero,G4,A4,E4,G4,F4,
        D4,E4,F4,D4,B4,A4,B4,G4,
        F4,zero,F4,zero,F4,G4,D4,F4,E4,
        A3,A3,B3,C4,D4,A4,zero,G4,B3,D4,C4,zero,
        A3,E4,E4,E4,A4,G4,F4,E4,F4,F4,F4,D4,zero,
        B3,D4,D4,D4,G4,F4,E4,D4,E4,E4,E4,C4,zero,
        C4,C5,A4,A4,G4,A4,B4,B4,A4,B4,zero,
        B3,B4,G4s,G4s,F4s,G4s,A4,A4,G4,A4,
        A3,B3,C4,E3,G4,G4,F4,G4,zero,
        A4,A4,G4,A4,B4,B4,zero,G4,C5,zero
};
unsigned int duration_heimaojingzhang[] = {
        4, 4, 4, 4, 4, 8, 4, 6, 26,
        4, 4, 12, 4, 6, 2, 8, 24,
        4,4,4,4,4,8,4,6,26,
        6,2,4,4,8,8,4,8,8,8,28,12,
        4,4,4,4,4,4,4,4,4,4,4,4,16,
        4,4,4,4,4,4,4,4,4,4,4,4,24,
        6,2,20,4,4,4,8,8,4,16,4,
        6,2,20,4,4,4,8,8,4,12,
        8,8,8,8,6,2,4,4,8,
        6,2,4,4,8,8,8,8,24,32
};

unsigned int notes_huluwa[] = {
        C4,C4,E4,C4,C4,E4,
        A4,A4,A4,G4,A4,G4,C4,E4,
        C5,A4,A4,G4,A4,G4,C4,D4,
        B4,G4,E4,G4,
        C5,A4,A4,G4,G4,A4,A4,zero,G4,C4,E4,
        C5,A4,A4,G4,G4,A4,A4,zero,G4,C4,D4,
        E4,C4,A3,C4,
        E4,G4,A4,E4,G4,A4,
        C5,B4,G4,A4
};
unsigned int duration_huluwa[] = {
        8,8,16,4,8,20,
        8,8,4,4,8,4,8,20,
        4,4,4,4,16,4,8,20,
        20,5,8,32,
        8,6,2,4,4,4,4,4,8,4,16,
        8,6,2,4,4,4,4,4,8,4,16,
        20,4,8,32,
        4,8,20,4,8,20,
        20,4,8,32
};

uint32_t Frequency = 0;
uint32_t CLOCK = 10500000;
int i;

void noTone(void){
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    osDelay(30);
}

void playSong_jiaofu(void){
    for (i = 0; i < sizeof(notes_jiaofu)/sizeof(notes_jiaofu[0]); i++) {
        Frequency = CLOCK / notes_jiaofu[i]; // 计算定时器的重载值
        htim4.Instance->ARR = Frequency; // 设置定时器的重载值
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Frequency / 2);// 设置PWM的占空比50%
        osDelay(50*duration_jiaofu[i]);
        noTone();
    }
}

void playSong_heimaojingzhang(void){
    for (i = 0; i < sizeof(notes_heimaojingzhang)/sizeof(notes_heimaojingzhang[0]); i++) {
        Frequency = CLOCK / notes_heimaojingzhang[i]; // 计算定时器的重载值
        htim4.Instance->ARR = Frequency; // 设置定时器的重载值
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Frequency / 2);// 设置PWM的占空比50%
        osDelay(50*duration_heimaojingzhang[i]);
        noTone();
    }
}

void playSong_huluwa(void){
    for (i = 0; i < sizeof(notes_huluwa)/sizeof(notes_huluwa[0]); i++) {
        Frequency = CLOCK / notes_huluwa[i]; // 计算定时器的重载值
        htim4.Instance->ARR = Frequency; // 设置定时器的重载值
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Frequency / 2);// 设置PWM的占空比50%
        osDelay(50*duration_huluwa[i]);
        noTone();
    }
}
