#include "manque.h"

SqQueue Queue;//����һ�����У�����ѭ���洢100������

u8 process_que(QElemType res)
{
	static u8 Status_EnQueue=OK;//��ʼʱ������Ϊ�գ�����Ϊ�����״̬
	static u8 Status_DeQueue=OK;//���ӷ���״̬
	QElemType DeleteValue;//���ӵ�ֵ
	
	
	
	Queue.length=QueueLength(Queue);//������г���	
	
	if( Queue.length + 1 < MAXSIZE -1 )//����δ����Ϊ�˱����ж϶�����ʱ��ʧһ�����ݣ���ǰԤ��һ��λ�ø�������
	{
		Status_EnQueue=EnQueue(&Queue, res);//�ѱ������������
//		printf("Queue�������\r\n");
		
		return 1;
	}
	else
	{
		Status_EnQueue=EnQueue(&Queue, res);//�ѱ������������
//		printf("Queue���н�����ɾ����ͷ�����������\r\n");//ʱ�̱������µ�����
		Status_DeQueue=DeQueue(&Queue, &DeleteValue);//ɾ��һ��Ԫ�أ�ʹ�����ݿ��Լ������
		
		return 2;
	}
}

