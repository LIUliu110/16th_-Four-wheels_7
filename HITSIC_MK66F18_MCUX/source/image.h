/*
 * image.h
 *
 *  Created on: 2020年11月10日
 *      Author: liuhe
 */

#ifndef _IMAGE_H
#define _IMAGE_H
#include <stdio.h>
#include <stdlib.h>
//#include "cv.h"
//#include "highgui.h"
#include <math.h>
#include "image.h"

#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 113//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern int banmaxian_flag;//斑马线标志位

void head_clear(void);
void THRE(void);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void image_main();
void get_mid_line(void);

float get_error(void);//11.14添加
int ckeck_out_road(void);//11.14添加
void search_leftdown_point();
void search_rightdown_point();
void search_leftup_point();
void search_rightup_point();
void connect_line_plan();
void banmaxian();
void cross_judge();


void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
#endif //

