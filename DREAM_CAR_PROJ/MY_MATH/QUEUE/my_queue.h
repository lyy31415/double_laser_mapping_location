#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include "sys.h"
#include "stdio.h"
#include "stdlib.h"

typedef float QElemType;// 定义队列数据类型

#define OK 1
#define ERROR 0
#define MAXSIZE 12  //队列实际使用长度大2的值，(因为要留空一个位置表示队列已满，还有留空一个用来缓冲判断是否队列已满)


//循环队列顺序存储结构
typedef struct
{
	QElemType data[MAXSIZE];//存储10个元素，留空一个表示队列已满
	u16 front;//头指针
	u16 rear;//指向尾元素下一个位置的指针
	u16 length;
}SqQueue;

u8 InitQueue(SqQueue *Q);//初始化队列
u8 QueueLength(SqQueue Q);//求队列长度
u8 EnQueue(SqQueue *Q, QElemType e);//入队
u8 DeQueue(SqQueue *Q, QElemType *e);//出队
QElemType AveQueue(SqQueue *Q);//求队列元素均值
QElemType HeadQueue(SqQueue *Q);//求队列头元素
QElemType TailQueue(SqQueue *Q);//求队列尾元素
QElemType process_que(QElemType res, SqQueue *Queue);

#endif

