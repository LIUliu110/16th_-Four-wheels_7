/*
 * control.cpp
 *
 *  Created on: 2020年12月5日
 *      Author: liuhe
 */

//#include "control.hpp"
//
//bool delay_runcar = 0;//延迟发车标志位
//float error_n = 0;      //舵机偏差
//float error_n_1 = 0;    //舵机前一次偏差
//int32_t mot_left = 0;  //电机左轮编码器读取
//int32_t mot_right = 0; //电机右轮编码器读取
//int32_t mora_flag = 0;
//
//float mot_err_l = 0;   //左电机偏差值
//float mot_err1_l = 0;  //左电机前一次偏差值
//
//float mot_err_r = 0;   //右电机偏差值
//float mot_err1_r = 0;  //右电机前一次偏差值
//
//float M_left_pwm = 0;  //左电机pwm值
//float M_right_pwm = 0;   //右电机pwm值
//
//float M_left_drs = 0;    //左电机理想速度
//float M_right_drs = 0;    //右电机理想速度
//
//cardata c_data[2]=
//{
//        {{22,0,150},7.55,7.55,0.019,0.012,0.020,0.010,1.0},
//        {{22,0,150},7.55,7.55,0.019,0.012,0.020,0.010,1.0},
//};
//void Motor_ctr(void)//电机控制闭环
//{
//
//    mot_left =  SCFTM_GetSpeed(FTM1);
//    SCFTM_ClearSpeed(FTM1);//测试差速时可以注释掉
//    mot_right = -SCFTM_GetSpeed(FTM2);
//    SCFTM_ClearSpeed(FTM2);//测试差速时可以注释掉
//
//    Motor_pid();
//
//    /*限幅代码*/
//    float *p;
//    if(M_left_pwm>45.0) {p = &M_left_pwm;*p = 45.0;}
//    else if(M_left_pwm<-45.0) {p = &M_left_pwm;*p = -45.0;}
//    else p = NULL;
//    if(M_right_pwm>45.0) {p = &M_right_pwm;*p = 45.0;}
//    else if(M_right_pwm<-45.0) {p = &M_right_pwm;*p = -45.0;}
//    else p = NULL;
//    /*限幅代码*/
//    /*if((ADC[1]<=40&&ADC[7]<=40)||delay_runcar==0)
//    {
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, 0U);//右轮正转kFTM_Chnl_0> kFTM_Chnl_1
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0U);
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0U);
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, 0U);//左轮正转kFTM_Chnl_3> kFTM_Chnl_2
//    }
//    else*/
//    if(M_right_pwm>0)
//    {
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U,M_right_pwm);//右轮正转kFTM_Chnl_0> kFTM_Chnl_1
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0U);
//    }
//    else
//    {
//     SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, 0U);//右轮反转kFTM_Chnl_0> kFTM_Chnl_1
//     SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, -M_right_pwm);
//    }
//    if(M_left_pwm>0)
//    {
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, 0U);
//    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, M_left_pwm);//左轮正转kFTM_Chnl_3> kFTM_Chnl_2
//    }
//    else
//    {
//        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, -M_left_pwm);
//        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0U);//左轮反转kFTM_Chnl_3> kFTM_Chnl_2
//    }
//
//}
//
//void Motorsp_Init()
//{
//    float *p_sp;
//    p_sp = &M_left_drs;
//    *p_sp = (float)c_data[0].Motorspeed[0];
//    p_sp = &M_right_drs;
//    *p_sp = (float)c_data[0].Motorspeed[0];
//
//}
//
//
//void Motor_pid()
//{
//    float *p_pwm,*p_erro,*p_errolast,*p_drs,*m_pwm;
//    p_erro = &mot_err_l;
//    p_drs = &M_left_drs;
//    p_pwm = &M_left_pwm;
//    p_errolast = &mot_err1_l;
//    *p_erro = *p_drs-(float)mot_left;//左电机偏差
//    *p_pwm += c_data[0].M_Kp*((*p_erro)-(*p_errolast))+c_data[0].M_Ki*(*p_erro);//左电机增量式
//    *p_errolast = *p_erro;//记录上一次偏差左
//
//    p_erro = &mot_err_r;
//    p_drs = &M_right_drs;
//    p_pwm = &M_right_pwm;
//    p_errolast = &mot_err1_r;
//    *p_erro = *p_drs-(float)mot_right;//右电机偏差
//    *p_pwm += c_data[0].M_Kp*((*p_erro)-(*p_errolast))+c_data[0].M_Ki*(*p_erro);//右电机增量式
//    *p_errolast = *p_erro;//记录上一次偏差右
//    /*限幅代码*/
//    if(M_left_pwm>45.0) {m_pwm = &M_left_pwm;*m_pwm = 45.0;}
//    else if(M_left_pwm<-45.0) {m_pwm = &M_left_pwm;*m_pwm = -45.0;}
//    else m_pwm = NULL;
//    if(M_right_pwm>45.0) {m_pwm = &M_right_pwm;*m_pwm = 45.0;}
//    else if(M_right_pwm<-45.0) {m_pwm = &M_right_pwm;*m_pwm = -45.0;}
//    else m_pwm = NULL;
//    /*限幅代码*/
//}
//void Motorsp_Set(float x,float y)
//{
//    float *p;
//    p = &M_left_drs;
//    *p = x;
//    p = &M_right_drs;
//    *p = y;
//
//}
//void Speed_radio(float x)
//{
//    float fa,a;
//       (x<0)?(a = -x):(a=x);
//       fa = (c_data[0].Sradio)*(0.2274*pow(a,3)-0.05485*pow(a,2)+0.7042*a)+1.018;
//       if(mora_flag%2==0)
//       (x>0)?(Motorsp_Set(((float)(c_data[0].Motorspeed[0]/fa)),((float)c_data[0].Motorspeed[0]))):(Motorsp_Set((float)(c_data[0].Motorspeed[0]),((float)(c_data[0].Motorspeed[0]/fa))));
//       else
//       (x>0)?(Motorsp_Set(((float)(c_data[0].Motorspeed[0]*2/(fa+1.0))),((float)(c_data[0].Motorspeed[0]*2*fa/(fa+1.0))))):(Motorsp_Set((float)(c_data[0].Motorspeed[0]*2*fa/(fa+1.0)),((float)(c_data[0].Motorspeed[0]*2/(fa+1.0)))));
//
//}



