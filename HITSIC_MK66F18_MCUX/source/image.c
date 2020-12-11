/*
 * image.c
 *
 *  Created on: 2020年11月10日
 *      Author: liuhe
 */
#include "image.h"
#include"my_math.h"

int f[10 * CAMERA_H];//考察连通域联通性

//每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    int   connect_num;//连通标记（号）
}range;

//每行的所有白条子
typedef struct {
    uint8_t   num;//每行白条数量
    range   area[white_num_MAX];//该行各白条区域
}all_range;

//属于赛道的每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   width;//宽度
}road_range;

//每行属于赛道的每个白条子
typedef struct {
    uint8_t   white_num;
    road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//所有白条子
road my_road[CAMERA_H];//赛道
uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
int all_connect_num = 0;//所有白条子数
uint8_t top_road;//赛道最高处所在行数
uint8_t* fullBuffer;

uint8_t threshold = 230;//阈值
int foresight = 80;//前瞻数值
int banmaxian_flag=0;//斑马线标志位
int cross_flag=0;//十字的标志位

//12.10定义的点结构体
typedef struct {
    int x;
    int y;
}point;

//11.19添加，确定的左上下，右上下四个点
point determined_leftdown_point;
point determined_leftup_point;
point determined_rightdown_point;
point determined_rightup_point;

////////////////////////////////////////////
//功能：二值化
//输入：灰度图片
//输出：二值化图片
//备注：
///////////////////////////////////////////
void THRE()
{
    uint8_t* map;
    uint8_t* my_map;
    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > threshold)
                (*my_map) = 255;
            else (*my_map) = 0;
            map++;
            my_map++;
        }
    }
}

////////////////////////////////////////////
//功能：粗犷的清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)
{
    uint8_t* my_map;
    for (int i = 119; i >= 84; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 40; j <= 135; j++)
        {
            *(my_map+j) = 255;
        }
    }
}

////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_f(int node)//返回的是父节点
{
    if (f[node] == node)return node;//找到最古老祖先，return
    f[node] = find_f(f[node]);//向上寻找自己的父节点
    return f[node];
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void search_white_range()
{
    uint8_t i, j;
    int istart = NEAR_LINE;//处理起始行
    int iend = FAR_LINE;//处理终止行
    int tnum = 0;//当前行白条数
    all_connect_num = 0;//白条编号初始化
    uint8_t* map = NULL;
    for (i = istart; i >= iend; i--)
    {
        map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
        tnum = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//遇白条左边界
            {
                tnum++;
                if (tnum >= white_num_MAX)break;
                range* now_white = &white_range[i].area[tnum];
                now_white->left = j;

                //开始向后一个一个像素点找这个白条右边界
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
                now_white->connect_num = ++all_connect_num;//白条数加一，给这个白条编号
            }
        }
        white_range[i].num = tnum;
    }
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
    //f数组初始化
    for (int i = 1; i <= all_connect_num; i++)
        f[i] = i;

    //u为up d为down 即为当前处理的这两行中的上面那行和下面那行
    //u_num：上面行白条数
    //u_left：上面行当前白条左边界
    //u_right：上面行当前白条右边界
    //i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    all_range* u_white = NULL;
    all_range* d_white = NULL;
    for (int i = NEAR_LINE; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
    {
        u_num = white_range[i - 1].num;
        d_num = white_range[i].num;
        u_white = &white_range[i - 1];
        d_white = &white_range[i];
        i_u = 1; i_d = 1;

        //循环到当前行或上面行白条子数耗尽为止
        while (i_u <= u_num && i_d <= d_num)
        {
            //变量先保存，避免下面访问写的冗杂且访问效率低
            u_left = u_white->area[i_u].left;
            u_right = u_white->area[i_u].right;
            d_left = d_white->area[i_d].left;
            d_right = d_white->area[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
                f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//父节点连起来

            //当前算法规则，手推一下你就知道为啥这样了
            if (d_right > u_right)i_u++;
            if (d_right < u_right)i_d++;
            if (d_right == u_right) { i_u++; i_d++; }
        }
    }
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road()
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
    int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= white_range[istart].num; i++)
        if (white_range[istart].area[i].left <= CAMERA_W / 2
            && white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 90)
            road_f = find_f(white_range[istart].area[i].connect_num);

    if (road_f == -1)//若赛道没在中间，在113行选一行最长的认为这就是赛道
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= white_range[istart].num; i++)
            if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
            {
                widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
                jselect = i;
            }
        road_f = find_f(white_range[istart].area[jselect].connect_num);
    }

    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                top_road = i;
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;

            }
        }
    }
}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分对多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t dright, dleft, uright, uleft;
    j_return = MISS;//如果没找到，输出255
    if (j_start > my_road[i_start].white_num)
        return MISS;
    //选一个重叠最大的
    for (j = 1; j <= my_road[i_start - 1].white_num; j++)
    {
        dleft = my_road[i_start].connected[j_start].left;
        dright = my_road[i_start].connected[j_start].right;
        uleft = my_road[i_start - 1].connected[j].left;
        uright = my_road[i_start - 1].connected[j].right;
        if (//相连
            dleft < uright
            &&
            dright > uleft
            )
        {
            //计算重叠大小
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////
void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//第一条连通路径
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;

    //寻找起始行最宽的白条子
    i_start = NEAR_LINE;
    i_end = FAR_LINE;
    width_max = 0;
    for (j = 1; j <= my_road[i_start].white_num; j++)
    {
        if (my_road[i_start].connected[j].width > width_max)
        {
            width_max = my_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    //记录连贯区域编号
    for (i = i_start; i > i_end; i--)
    {
        //如果相连编号大于该行白条数，非正常，从此之后都MISS
        if (j_continue[i] > my_road[i].white_num)
        {
            j_continue[i - 1] = MISS;
        }
        else
        {
            j_continue[i - 1] = find_continue(i, j_continue[i]);
        }

    }

    //全部初始化为MISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);


    for (i = i_start; i > i_end; i--)
    {
        if (j_continue[i] <= my_road[i].white_num)
        {
            left_line[i] = my_road[i].connected[j_continue[i]].left;
            right_line[i] = my_road[i].connected[j_continue[i]].right;
            IMG[i][left_line[i]] = blue;
            IMG[i][right_line[i]] = red;
        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
}

////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}
////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);
    for(int i = NEAR_LINE;i >= FAR_LINE;i--)
        if (left_line[i] != MISS)
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
        }
        else
        {
            mid_line[i] = MISS;
        }

}
////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
void image_main()
{
    search_white_range();
    find_all_connect();
    find_road();
    banmaxian();
    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
    ordinary_two_line();

    //为十字
    if (cross_flag==1)
    {
        search_leftdown_point();
        search_rightdown_point();
        search_leftup_point();
        search_rightup_point();
       connect_line_plan();
        for (int i = 3; i < 115; i++)
        {
            for (int j = 3; j < 185; j++)
            {
                if (IMG[i][j] == 11)
                    left_line[i] = j;
                if (IMG[i][j] == 12)
                    right_line[i] = j;
            }
        }
       }

    get_mid_line();

    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
        if (mid_line[i] != MISS)
            IMG[i][mid_line[i]] = 0;
}

float get_error()
{
    float a=94-mid_line[foresight];
    return a;
}
int ckeck_out_road(void)//检测跑出赛道函数
{
    uint8_t* my_map;
    int count = 0;//检测该范围黑点个数
    for (int i = 115; i >= 80; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 60; j <= 110; j++)
        {
            if (*(my_map + j) ==0)
            {
                //*(my_map + j) = blue;
                count++;
            }
        }
    }
    if (count >= 1500)
        return 0;
    else
        return 1;

}

//求取斜率
float check_k(int line, uint8_t* array, int length, int flag)
{
    float k = 0;
    int sumx = 0;
    int sumy = 0;
    int sumxy = 0;
    int sumx2 = 0;
    for (int i = 0; i < length; i++)
    {
        if (flag == 1)
        {
            sumx = sumx + line + i;
            sumy = sumy + array[line + i];
            sumxy = sumxy + (line + i)*array[line + i];
            sumx2 = sumx2 + (line + i) * (line + i);
        }
        if (flag == 0)
        {
            sumx = sumx + line - i;
            sumy = sumy + array[line - i];
            sumxy = sumxy + (line - i)*array[line - i];
            sumx2 = sumx2 + (line - i) * (line - i);
        }

    }
    k = -(1.0*sumxy - 1.0*sumx*sumy / length) / (1.0*sumx2 - 1.0*sumx*sumx / length);
    //k=my_arctan(k);
    return k;
}


//两点连线，flag是标志位
void  connect_line(int x1, int y1, int x2, int y2,int flag)
{
    float k, b; int x;
    point line_point;
    k = (y2 - y1)*1.0 / (x2 - x1);
    b = y1 - k * x1;
    for (x = my_min(x1,x2); x < my_max(x1, x2); x++)
    {
        line_point.x = x;
        line_point.y = k * x + b;
        IMG[line_point.x][line_point.y] = flag;
    }
}
//从确定的四个点，进一步确定不同情况下的连线方式，仅仅作为验证使用，即连线方案,这一步目前还有点bug
void connect_line_plan()
{
    //下边两点不是零

    //determined_leftup_point
    //determined_leftdown_point
    //determined_rightup_point
    //determined_rightdown_point

    int take_points = 5;//最小二乘法考查点数
    int star_position = 2; //最小二乘法起始点数，如 take_points = 5，star_position = 2;应该考察2——7点计算斜率k，对于下边的点适用
    //左边两个点都存在
    if ((determined_leftdown_point.x != 0) && (determined_leftup_point.x != 0))
        connect_line(determined_leftdown_point.x, determined_leftdown_point.y, determined_leftup_point.x, determined_leftup_point.y,11);
//    //左下点存在
//    if ((determined_leftdown_point.x != 0) && (determined_leftup_point.x == 0))
//    {
//        int line = 2; float y = 0; int y1 = 0;
//        float k = check_k(determined_leftdown_point.x+ star_position, left_line, take_points, 1);
//        y = (determined_leftdown_point.x - line)*k + determined_leftdown_point.y;
//        y1 = (int)y;
////        if ((y1 > 187) || (y1 < 0))
////      y1 = right_line[foresight];
//        connect_line(determined_leftdown_point.x, determined_leftdown_point.y, line, y1,11);
//    }
//
//    //左上点存在
//    if ((determined_leftdown_point.x == 0) && (determined_leftup_point.x != 0))
//    {
//        int line = 115; float y = 0; int y1 = 0;
//        float k = check_k(determined_leftup_point.x, left_line, take_points, 0);
//        y = (determined_leftup_point.x- line)*k + determined_leftup_point.y;
//        y1 = (int)y;
//        if ((y1 > 187) || (y1 < 0))
//            y1 = 0;
//        connect_line(determined_leftup_point.x, determined_leftup_point.y, line, y1,11);
//    }

    //右边两个点存在
    if ((determined_rightdown_point.x != 0) && (determined_rightup_point.x != 0))
        connect_line(determined_rightdown_point.x, determined_rightdown_point.y, determined_rightup_point.x, determined_rightup_point.y, 12);

//    //右下点存在,右上点不存在
//    if ((determined_rightdown_point.x != 0) && (determined_rightup_point.x == 0))
//    {
//        int line = 2; float y = 0; int y1 = 0;
//        float k = check_k(determined_rightdown_point.x+ star_position, right_line, take_points, 1);
//        y = -(line- determined_rightdown_point.x)*k + determined_rightdown_point.y;
//        y1 = (int)y;
//        if ((y1 > 187) || (y1 < 0))
//            y1 = right_line[foresight];
//
//        connect_line(determined_rightdown_point.x, determined_rightdown_point.y, line, y1, 12);
//    }

//    //右上点存在
//    if ((determined_rightdown_point.x == 0) && (determined_rightup_point.x != 0))
//    {
//        int line = 115; float y = 0; int y1 = 0;
//        float k = check_k(determined_rightup_point.x, right_line, take_points, 0);
//        y = (line - determined_rightdown_point.x)*k + determined_rightdown_point.y;
//        y1 = (int)y;
//        if ((y1 > 187) || (y1 < 0))
//            y1 = 187;
//        connect_line(determined_rightup_point.x, determined_rightup_point.y, line, y1,12);
//    }

}
//寻找左下右下点粗暴的方法，但是很有用
void search_leftdown_point()
{
    determined_leftdown_point.x = 0;
    determined_leftdown_point.y = 0;
    int left_max = left_line[foresight];
    for (int i = foresight; i < 115; i++)
    {
        if (left_line[i] >= left_max)
        {
            determined_leftdown_point.x = i;
            determined_leftdown_point.y = left_line[i];
            left_max = left_line[i];
        }
    }
    if (determined_leftdown_point.x == 0 && determined_leftdown_point.y == 0)
    {
        determined_leftdown_point.x = 115;
        determined_leftdown_point.y = left_line[115];
    }

    if (determined_leftdown_point.y < 2)
    {
        determined_leftdown_point.x = 115;
        determined_leftdown_point.y = left_line[115];
    }
}
void search_rightdown_point()
{
    determined_rightdown_point.x = 0;
    determined_rightdown_point.y = 0;
    int right_min = right_line[foresight];
    for (int i = foresight; i < 115; i++)
    {
        if (right_line[i] <= right_min)
        {
            determined_rightdown_point.x = i;
            determined_rightdown_point.y = right_line[i];
            right_min = right_line[i];
        }
    }
    if (determined_rightdown_point.x == 0 && determined_rightdown_point.y == 0)
    {
        determined_rightdown_point.x = 115;
        determined_rightdown_point.y = right_line[115];
    }

    if (determined_rightdown_point.y < 2)
    {
        determined_rightdown_point.x = 115;
        determined_rightdown_point.y = right_line[115];
    }
}
void  search_leftup_point()
{
    determined_leftup_point.x = 0;
    determined_leftup_point.y = 0;
    for (int i = 5; i < foresight; i++)
    {
        if ((left_line[i] - left_line[i+2])>10&& (left_line[i-2]- left_line[i])<=4&& (left_line[i - 2]  -left_line[i])>=0)
        {
            determined_leftup_point.x = i;
            determined_leftup_point.y = left_line[i];
        }
    }

    if (determined_leftup_point.y >= determined_rightdown_point.y)
    {
        determined_leftup_point.x = 0;
        determined_leftup_point.y = 0;
    }
}
void search_rightup_point()
{
    determined_rightup_point.x = 0;
    determined_rightup_point.y = 0;
    for (int i = 5; i <foresight; i++)
    {
        if ((right_line[i] - right_line[i + 2]) < -10 && (right_line[i]- right_line[i-2])>=0&& (right_line[i] - right_line[i - 2]) <=4)
        {
            determined_rightup_point.x = i;
            determined_rightup_point.y = right_line[i];
        }

        if (determined_rightup_point.y < determined_leftdown_point.y)
        {
            determined_rightup_point.x = 0;
            determined_rightup_point.y = 0;
        }
    }
}

void banmaxian()
{
    int count_flag = 0;

    if ((my_road[foresight].white_num > 5)
       || (my_road[foresight - 2].white_num) > 5
       || (my_road[foresight - 4].white_num) > 5
       //|| (my_road[foresight + 2].white_num) > 5
       //|| (my_road[foresight + 4].white_num) > 5
       )
    {
        if((my_road[foresight - 1].white_num = 1))
        count_flag ++;
    }

    if((my_road[foresight].white_num > 5)
       || (my_road[foresight - 2].white_num) > 5
       || (my_road[foresight - 4].white_num) > 5)
    {
        if(count_flag == 1)
        {
            banmaxian_flag = 1;
        }

    }
}
void cross_judge()
{
    cross_flag=0;
    if (
          my_road[foresight].white_num == 1 &&
          (
          (my_road[foresight].connected[1].width)>160
          ||(my_road[foresight-2].connected[1].width) > 160
          ||(my_road[foresight-4].connected[1].width) > 160
          ||(my_road[foresight-6].connected[1].width) > 160
          || (my_road[foresight - 8].connected[1].width) > 160
          || (my_road[foresight+2].connected[1].width) > 160
          || (my_road[foresight + 4].connected[1].width) > 160
          //|| (my_road[foresight + 6].connected[1].width) > 160
          )
          )
        cross_flag=1;
    else
        cross_flag=0;
}
