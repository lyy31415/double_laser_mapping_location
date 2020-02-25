#ifndef __HEAPSORT_H
#define __HEAPSORT_H


#include "sys.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#include "arm_math.h"
#include "my_queue.h"
#include "manque.h"


//#define HEAP_MAXSIZE 100
//typedef struct
//{
//	float r[HEAP_MAXSIZE+1];//用于存储要排序数组，r[0]用作哨兵或临时变量
//	int length;
//}SqList;

#define SqList SqQueue

void swap(SqList *L,int i,int j);
void HeapAdjust(SqList *L, int s, int m);
void HeapSort(SqList *L);

#endif

