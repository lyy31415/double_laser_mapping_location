#include "my_queue.h"


//初始化一个空队列Q
u8 InitQueue(SqQueue *Q)
{
	u16 i=0;
	Q->front=0;
	Q->rear=0;
	Q->length=(Q->rear-Q->front+MAXSIZE)%MAXSIZE;
	
	for(i=0;i<MAXSIZE;i++)
	{
		Q->data[i]=0;//队列初始化为0.0
	}
	
	
	printf("队列初始化成功\r\n");
	
	return OK;//初始化成功
}


//求队列当前长度
u8 QueueLength(SqQueue Q)
{
	return (Q.rear-Q.front+MAXSIZE)%MAXSIZE;
}


//循环队列的入队操作，若队列未满，则插入元素e为Q新的队尾元素
u8 EnQueue(SqQueue *Q, QElemType e)
{
	if((Q->rear+1)%MAXSIZE == Q->front)//队列满的判断
	{
		return ERROR;
	}
	
	Q->data[Q->rear]=e;//把元素e放到队尾
	Q->rear=(Q->rear+1)%MAXSIZE;//rear指针向后移动一位，若到最后则转到数组头部
	
//	printf("入队成功\r\n");
	
	return OK;
}

u8 DeQueue(SqQueue *Q, QElemType *e)
{
	if(Q->front == Q->rear)//队列为空的判断
	{
		return ERROR;
	}
	
	*e=Q->data[Q->front];//将队头元素赋给e
	Q->data[Q->front]=0;
	Q->front=(Q->front+1)%MAXSIZE;//front指针向后移动一位，若到最后则转到数组头部
	
//	printf("出队成功\r\n");
	
	return OK;
}


//求已入队元素平均值
QElemType AveQueue(SqQueue *Q)
{
	u16 i_sum=0;
	QElemType sum_data=0.0;
	QElemType ave_data=0.0;
	u16 len_queue=0;
	
	for(i_sum=0;i_sum<MAXSIZE;i_sum++)
	{
		sum_data+=Q->data[i_sum];//求队列Q元素之和，没有值的位置是0
	}
		
	len_queue=QueueLength(*Q);//求队列长度
	
	ave_data=sum_data / len_queue;//求队列平均值
	
//	printf("求平均成功\r\n");
	
	return ave_data;
}


//求队列头元素
QElemType HeadQueue(SqQueue *Q)
{
	QElemType head;
	
	head=Q->data[Q->front];
	
	return head;
}




//求队列尾元素
QElemType TailQueue(SqQueue *Q)
{
	QElemType tail;
	
	tail=Q->data[(Q->rear - 1 + MAXSIZE) % MAXSIZE];
	
	return tail;
}



QElemType process_que(QElemType res, SqQueue *Queue)
{
	static u8 Status_EnQueue=OK;//初始时，队列为空，设置为可入队状态
	static u8 Status_DeQueue=OK;//出队返回状态
	QElemType DeleteValue;//出队的值
	QElemType val;
	
	
	Queue->length=QueueLength(*Queue);//计算队列长度	
	
	if( Queue->length + 1 < MAXSIZE -1 )//队列未满，为了避免判断队列满时损失一个数据，提前预留一个位置给此数据
	{
		Status_EnQueue=EnQueue(Queue, res);//把编码器数据入队
//		printf("Queue队列入队\r\n");
	}
	else
	{
		Status_EnQueue=EnQueue(Queue, res);//把编码器数据入队
//		printf("Queue队列将满，删除队头，以重新入队\r\n");//时刻保持最新的数据
		Status_DeQueue=DeQueue(Queue, &DeleteValue);//删除一个元素，使得数据可以继续入队
	}
	
	val = AveQueue(Queue);
	
	return val;
}





