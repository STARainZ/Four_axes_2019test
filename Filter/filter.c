/**
  ******************************************************************************
  * �ļ�����: filter.c
  * ��    ��: By Sw Young
  * ��    ��: V1.0
  * ��    ��:
  * ��д����: 2018.7.6
  ******************************************************************************
  * ˵����
  * Ӳ��ƽ̨��TM4C123G
  *   *****
  * ������˵����
  *   *****
  * Github��
  ******************************************************************************
**/

#include "filter.h"
#include "head.h"
#define Diff 20
uint16_t Value = 0;

/**
  * �� �� ��: swap
  * ��������: ��������ֵ
  * �������: ��Ҫ����������ֵ
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * �� �� ��: SelectSort
  * ��������: ���������ѡ������
  * �������: ѡӴ��������顢���鳤��
  * �� �� ֵ: ��
  * ˵    ��: ��
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
//�޷�����λֵ�˲�
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
