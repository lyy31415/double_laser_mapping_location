#include "my_queue.h"


//��ʼ��һ���ն���Q
u8 InitQueue(SqQueue *Q)
{
	u16 i=0;
	Q->front=0;
	Q->rear=0;
	Q->length=(Q->rear-Q->front+MAXSIZE)%MAXSIZE;
	
	for(i=0;i<MAXSIZE;i++)
	{
		Q->data[i]=0;//���г�ʼ��Ϊ0.0
	}
	
	
	printf("���г�ʼ���ɹ�\r\n");
	
	return OK;//��ʼ���ɹ�
}


//����е�ǰ����
u8 QueueLength(SqQueue Q)
{
	return (Q.rear-Q.front+MAXSIZE)%MAXSIZE;
}


//ѭ�����е���Ӳ�����������δ���������Ԫ��eΪQ�µĶ�βԪ��
u8 EnQueue(SqQueue *Q, QElemType e)
{
	if((Q->rear+1)%MAXSIZE == Q->front)//���������ж�
	{
		return ERROR;
	}
	
	Q->data[Q->rear]=e;//��Ԫ��e�ŵ���β
	Q->rear=(Q->rear+1)%MAXSIZE;//rearָ������ƶ�һλ�����������ת������ͷ��
	
//	printf("��ӳɹ�\r\n");
	
	return OK;
}

u8 DeQueue(SqQueue *Q, QElemType *e)
{
	if(Q->front == Q->rear)//����Ϊ�յ��ж�
	{
		return ERROR;
	}
	
	*e=Q->data[Q->front];//����ͷԪ�ظ���e
	Q->data[Q->front]=0;
	Q->front=(Q->front+1)%MAXSIZE;//frontָ������ƶ�һλ�����������ת������ͷ��
	
//	printf("���ӳɹ�\r\n");
	
	return OK;
}


//�������Ԫ��ƽ��ֵ
QElemType AveQueue(SqQueue *Q)
{
	u16 i_sum=0;
	QElemType sum_data=0.0;
	QElemType ave_data=0.0;
	u16 len_queue=0;
	
	for(i_sum=0;i_sum<MAXSIZE;i_sum++)
	{
		sum_data+=Q->data[i_sum];//�����QԪ��֮�ͣ�û��ֵ��λ����0
	}
		
	len_queue=QueueLength(*Q);//����г���
	
	ave_data=sum_data / len_queue;//�����ƽ��ֵ
	
//	printf("��ƽ���ɹ�\r\n");
	
	return ave_data;
}


//�����ͷԪ��
QElemType HeadQueue(SqQueue *Q)
{
	QElemType head;
	
	head=Q->data[Q->front];
	
	return head;
}




//�����βԪ��
QElemType TailQueue(SqQueue *Q)
{
	QElemType tail;
	
	tail=Q->data[(Q->rear - 1 + MAXSIZE) % MAXSIZE];
	
	return tail;
}



QElemType process_que(QElemType res, SqQueue *Queue)
{
	static u8 Status_EnQueue=OK;//��ʼʱ������Ϊ�գ�����Ϊ�����״̬
	static u8 Status_DeQueue=OK;//���ӷ���״̬
	QElemType DeleteValue;//���ӵ�ֵ
	QElemType val;
	
	
	Queue->length=QueueLength(*Queue);//������г���	
	
	if( Queue->length + 1 < MAXSIZE -1 )//����δ����Ϊ�˱����ж϶�����ʱ��ʧһ�����ݣ���ǰԤ��һ��λ�ø�������
	{
		Status_EnQueue=EnQueue(Queue, res);//�ѱ������������
//		printf("Queue�������\r\n");
	}
	else
	{
		Status_EnQueue=EnQueue(Queue, res);//�ѱ������������
//		printf("Queue���н�����ɾ����ͷ�����������\r\n");//ʱ�̱������µ�����
		Status_DeQueue=DeQueue(Queue, &DeleteValue);//ɾ��һ��Ԫ�أ�ʹ�����ݿ��Լ������
	}
	
	val = AveQueue(Queue);
	
	return val;
}





