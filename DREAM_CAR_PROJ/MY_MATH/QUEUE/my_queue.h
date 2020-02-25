#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include "sys.h"
#include "stdio.h"
#include "stdlib.h"

typedef float QElemType;// ���������������

#define OK 1
#define ERROR 0
#define MAXSIZE 12  //����ʵ��ʹ�ó��ȴ�2��ֵ��(��ΪҪ����һ��λ�ñ�ʾ������������������һ�����������ж��Ƿ��������)


//ѭ������˳��洢�ṹ
typedef struct
{
	QElemType data[MAXSIZE];//�洢10��Ԫ�أ�����һ����ʾ��������
	u16 front;//ͷָ��
	u16 rear;//ָ��βԪ����һ��λ�õ�ָ��
	u16 length;
}SqQueue;

u8 InitQueue(SqQueue *Q);//��ʼ������
u8 QueueLength(SqQueue Q);//����г���
u8 EnQueue(SqQueue *Q, QElemType e);//���
u8 DeQueue(SqQueue *Q, QElemType *e);//����
QElemType AveQueue(SqQueue *Q);//�����Ԫ�ؾ�ֵ
QElemType HeadQueue(SqQueue *Q);//�����ͷԪ��
QElemType TailQueue(SqQueue *Q);//�����βԪ��
QElemType process_que(QElemType res, SqQueue *Queue);

#endif

