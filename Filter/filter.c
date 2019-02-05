/**
  ******************************************************************************
  * 文件名程: filter.c
  * 作    者: By Sw Young
  * 版    本: V1.0
  * 功    能:
  * 编写日期: 2018.7.6
  ******************************************************************************
  * 说明：
  * 硬件平台：TM4C123G
  *   *****
  * 软件设计说明：
  *   *****
  * Github：
  ******************************************************************************
**/

#include "filter.h"
#include "head.h"
#define Diff 20
uint16_t Value = 0;

/**
  * 函 数 名: swap
  * 函数功能: 交换两个值
  * 输入参数: 需要交换的两个值
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.12.12
  */
void swap(double *left, double *right)
{
    double temp = *left;
    *left = *right;
    *right = temp;
}
/**
  * 函 数 名: SelectSort
  * 函数功能: 对数组进行选择排序
  * 输入参数: 选哟排序的数组、数组长度
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.12.12
  */
void SelectSort(double arr[], int num)
{
    int i, j, Mindex;
    for (i = 0; i < num; i++)
    {
        Mindex = i;
        for (j = i + 1; j < num; j++)
        {
            if (arr[j] < arr[Mindex])
                Mindex = j;
        }

        swap(&arr[i], &arr[Mindex]);
    }
}
//限幅、中位值滤波
double LimitingFilter(double Array[],uint8_t Num)
{
    uint8_t i,j,k;
    double Avg = 0;
    SelectSort(Array,Num);
    for(i = 0;Array[Num/2]-Array[i]>Diff;i++)
    for(j = Num-1;Array[j]-Array[Num/2]>Diff;j--);

    for(k=i;k<=j;k++)
    {
        Avg += Array[k];
    }
    return Avg = Avg/(j-i+1);
}
