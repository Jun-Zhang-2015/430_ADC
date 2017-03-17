/***********
*		sinTab(int i);
************/
#include <math.h>


#define TAB_N 250                                           //����ʵ�ʲ��ĵ���
#define PI 3.1415926535897932384626433832795028841971       //����Բ����ֵ
float  sin_tab[TAB_N+1];                                 //��ʼ��1/4��sin��(256��)

//unsigned int max_sampels;												//  samples �������������ȷ���ģ�����0.5Hz�ֱ��ʣ��ز�Ƶ��5K��
																								//	�ز����ڲ���ʱ������0.5Hzʱ������Ϊ50*100/0.5=10000;
unsigned int steps;															//  =samples/TAB_N  ,����ʵ�������ֵ���ƣ�
unsigned int total_samples;
unsigned int h_samples;
unsigned int q_samples;
/*
*				msamples:   һ�����ڵĳ�������
*/

void init_sinTab(unsigned int msamples)					//  msamples == 500 000  ʱ֧��0.1hz/5k  ���ˣ�
{
			int i;

			for (i=0;i<=TAB_N;i++)
			{
				sin_tab[i]=sin(i*PI/2/TAB_N);
			}
			total_samples = msamples;
			h_samples = total_samples>>1;
			q_samples = h_samples>>1;
			steps = q_samples/TAB_N;								//   !!!�������������ɳ���Ա�˹����ƣ��мǣ�
}


/*********************************************************************************
����ԭ�ͣ�float sinTab(unsigned int i)
�������ܣ����ò�������ģ��ķ�������һ���� i ������ֵ,
���������i�仯ֵΪ ��0--samples����samplesӦ���ΪTAB_N*4�����������൱�� 0 - 2*PI
�����������Ӧ�������ֵ
**********************************************************************************/
float sinTab(unsigned int i)
{
    int n;
    float a,j;

    if( i >= h_samples)                // 0 ~ PI/2
    {
        i = i-h_samples;
        if ( i>= q_samples ) 
        		i=h_samples-i;
        n = i/steps;
        j = i%steps;	
        a=-sin_tab[n]-j/steps *(sin_tab[n+1]-sin_tab[n]);
		}
		else {
				if (i >= q_samples )
					i = h_samples - i;
				n = i/steps;
				j = i%steps;
				a = sin_tab[n]-j/steps *(sin_tab[n+1]-sin_tab[n]);
		}	
    return a;
}