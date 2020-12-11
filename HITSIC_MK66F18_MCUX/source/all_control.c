/*
 * all_control.c
 *
 *  Created on: 2020年12月6日
 *      Author: liuhe
 */

#include"all_control.h"
#include "sc_ftm.h"
#include "image.h"
#include "hitsic_common.h"
float error_n_=0;
float error_n_1_=0;
const float servo_mid=7.45;
float servo_pwm=7.45;
float motor_speed=15.0;
float motor_speed_now;
float P = 0.015,  D = 0.01;
float data[5]={20,30,40};
int counter1=0,counter2=0;
int mark=0;
float fang=30;

int delay_runcar=0;

int32_t mot_left = 0;           //电机左轮编码器读取值
int32_t mot_right = 0;          //电机右轮编码器读取值

float mot_err_l = 0;            //左电机编码器偏差值
float mot_err1ast_l = 0;           //左电机编码器前一次偏差值

float mot_err_r = 0;            //右电机编码器偏差值
float mot_err1ast_r = 0;           //右电机编码器前一次偏差值

float M_left_drs=60;          //左电机编码器理想值，对应着某一个理想速度
float M_right_drs=60;         //右电机编码器理想值，对应着某一个理想速度

float M_Kp=2;
float M_Ki=2;

float M_pwm_max = 50;           //限幅值
float M_left_pwm = 0;           //左电机pwm值
float M_right_pwm = 0;          //右电机pwm值


//产生方波的测试
void motor_test(){
    if(mark==0){
//            Motorsp_Set(fang,fang);
            fang=-20;
            M_left_drs = fang;
            M_right_drs = fang;
            mark=1;
     }
    else if(mark==1){
//            Motorsp_Set(fang,fang);
            fang=20;
            M_left_drs = fang;
            M_right_drs = fang;
            mark=0;
     }
}

void delay_run(){
    if(GPIO_PinRead(GPIOA,9)==1){
        delay_runcar=0;
    }
    else{


    }


}


void servo_pid()
{
    float pwm_error=0;
    error_n_=get_error();
    pwm_error=P*error_n_+D*(error_n_-error_n_1_);
    servo_pwm=servo_mid+pwm_error;
    if(servo_pwm<6.8)
        servo_pwm=6.8;
    else if(servo_pwm>8.2)
        servo_pwm=8.2;

    error_n_1_=error_n_;
};

void motor(void)
{
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,0);//电机恒定速度输出
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,motor_speed);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,motor_speed);
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000,0);
}
void servo()
{
    SCFTM_PWM_ChangeHiRes(FTM3,kFTM_Chnl_7,50,servo_pwm);
}

void my_motor_pid()//电机pid控制算法
{
    //左电机pid控制
    mot_err_l=M_left_drs-(float)mot_left;
    M_left_pwm=M_Kp*(mot_err_l-mot_err1ast_l)+M_Ki*mot_err_l;
    mot_err1ast_l=mot_err_l;

    //右电机pid控制
    mot_err_r=M_right_drs-(float)mot_right;
    M_right_pwm=M_Kp*(mot_err_r-mot_err1ast_r)+M_Ki*mot_err_r;
    mot_err1ast_r=mot_err_r;

    //电机pwm限幅
    if(M_left_pwm>M_pwm_max)
        {M_left_pwm=M_pwm_max;}
    if(M_left_pwm<-M_pwm_max)
        {M_left_pwm=-M_pwm_max;}
    if(M_right_pwm>M_pwm_max)
        {M_right_pwm = M_pwm_max;}
    if(M_right_pwm<-M_pwm_max)
        {M_right_pwm =-M_pwm_max;}
}

void my_motor_ctr()//电机闭环控制
{
    mot_left =  SCFTM_GetSpeed(FTM1);
    SCFTM_ClearSpeed(FTM1);//测试差速时可以注释掉
    mot_right = -SCFTM_GetSpeed(FTM2);
    SCFTM_ClearSpeed(FTM2);//测试差速时可以注释掉
    if(banmaxian_flag == 1) {Motorsp_Set(0.0,0.0);my_motor_pid();}
    else my_motor_pid();

    if(delay_runcar== 0)//延迟发车
        {
            SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U,0U);
            SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0U);
            SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, 0U);
            SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0U);
        }//延时发车



    if(M_right_pwm>0)
    {//右轮正转
    SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_1,20000,M_right_pwm);
    SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U,0);
    }
    else
    {//右轮反转
     SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_0,20000,-M_right_pwm);
     SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U,0);
    }

    if(M_left_pwm>0)
    {
        //左轮正转kFTM_Chnl_3> kFTM_Chnl_2
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_2,20000,M_left_pwm);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U,0);
    }
    else
    {
        //左轮反转kFTM_Chnl_3> kFTM_Chnl_2
        SCFTM_PWM_ChangeHiRes(FTM0,kFTM_Chnl_3,20000,-M_left_pwm);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U,0);
    }
}


//给左右编码器赋值
void Motorsp_Set(float x,float y)
{
    M_left_drs=x;
    M_right_drs=y;
//    float *p;
//    p = &M_left_drs;
//    *p = x;
//    p = &M_right_drs;
//    *p = y;

}

//获取差速比
void Speed_radio(float x)
{
    float fa;//差速比例系数
    float a;//舵机当前pwm值减去舵机中值

    if(x<0)
        a = -x;
    else
        a=x;

    fa = (1.0)*(0.2274*pow(a,3)-0.05485*pow(a,2)+0.7042*a)+1.018;

    if(x>0)
       Motorsp_Set((M_left_drs/fa),M_left_drs);
    else
        Motorsp_Set(M_left_drs,(M_left_drs/fa));
}
