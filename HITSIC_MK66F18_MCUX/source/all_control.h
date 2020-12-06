/*
 * all_control.h
 *
 *  Created on: 2020年12月6日
 *      Author: liuhe
 */

#ifndef ALL_CONTROL_H_
#define ALL_CONTROL_H_

#include "hitsic_common.h"

extern float error_n_;
extern float error_n_1_;
extern const float servo_mid;
extern float servo_pwm;
extern float motor_speed;
extern float motor_speed_now;
extern float P ,  D ;
extern float data[5];
extern int counter1,counter2;
extern int mark;
extern float fang;
extern float motor_test_[2];

extern int32_t mot_left;  //电机左轮编码器读取
extern int32_t mot_right; //电机右轮编码器读取


extern float mot_err_l ;            //左电机编码器偏差值
extern float mot_err1ast_l ;           //左电机编码器前一次偏差值

extern float mot_err_r ;            //右电机编码器偏差值
extern float mot_err1ast_r ;           //右电机编码器前一次偏差值

extern float M_left_drs ;          //左电机编码器理想值，对应着某一个理想速度
extern float M_right_drs ;         //右电机编码器理想值，对应着某一个理想速度

extern float M_Kp;
extern float M_Ki;

extern float M_pwm_max;           //限幅值
extern float M_left_pwm;           //左电机pwm值
extern float M_right_pwm;          //右电机pwm值




void servo_pid();
void motor_test();
void servo();
void motor(void);

void my_motor_pid(void);//电机pid控制函数
void my_motor_ctr(void);//电机闭环控制


#endif /* ALL_CONTROL_H_ */
