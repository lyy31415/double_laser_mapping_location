#include "manque.h"

SqQueue Queue;//创建一个队列，用于循环存储100组数据

u8 process_que(QElemType res)
{
	static u8 Status_EnQueue=OK;//初始时，队列为空，设置为可入队状态
	static u8 Status_DeQueue=OK;//出队返回状态
	QElemType DeleteValue;//出队的值
	
	
	
	Queue.length=QueueLength(Queue);//计算队列长度	
	
	if( Queue.length + 1 < MAXSIZE -1 )//队列未满，为了避免判断队列满时损失一个数据，提前预留一个位置给此数据
	{
		Status_EnQueue=EnQueue(&Queue, res);//把编码器数据入队
//		printf("Queue队列入队\r\n");
		
		return 1;
	}
	else
	{
		Status_EnQueue=EnQueue(&Queue, res);//把编码器数据入队
//		printf("Queue队列将满，删除队头，以重新入队\r\n");//时刻保持最新的数据
		Status_DeQueue=DeQueue(&Queue, &DeleteValue);//删除一个元素，使得数据可以继续入队
		
		return 2;
	}
}

