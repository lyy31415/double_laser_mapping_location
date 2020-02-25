#include "heapsort.h"

//���ڽ�������
void swap(SqList *L,int i,int j)
{
	int temp=L->data[j];
	L->data[j]=L->data[i];
	L->data[i]=temp;
}


//�Ѹ��ڵ�����Һ��ӽڵ������ֵ�͸��ڵ㻥��������ӽڵ�ֵ���
//�����������ӽڵ�һֱ�Ƚϲ�������ȥ    ע���˴�������Դ󶥶ѽ��еĲ���
void HeapAdjust(SqList *L, int s, int m)
{
	int temp,j;
	temp=L->data[s];//�Ѹ��ڵ��ֵ�ݴ��������Ա�����ڵ�ֵ����ʹ��
	
	for(j=2*s;j<=m;j*=2)
	{
		if(j<m && L->data[j]<L->data[j+1])//�ҳ������ӽڵ������ֵ
		{
			++j;//jΪ���ֵ�ڵ��±�
		}
		
		if(temp>=L->data[j])
		{
			break;//������ڵ��Ѿ����������ӽڵ㣬˵���Ѿ�����󶥶ѳɹ����˳�ѭ��
		}
		
		L->data[s]=L->data[j];
		s=j;
	}
	
	L->data[s]=temp;//�Ѹ��ڵ�ֵ���ڴ˴�����ɸ��ڵ������ӽڵ�����ݽ���
}



//�Խṹ��������Ԫ�ؽ��ж�����
void HeapSort(SqList *L)
{
	int i;
	for(i=L->length/2;i>0;i--)//��L��Ԫ�ع����һ���󶥶�
	{
		HeapAdjust(L,i,L->length);
	}
	
	for(i=L->length;i>1;i--)
	{
		swap(L,1,i);//�ѶѶ�Ԫ�غ����һ��Ԫ�ػ���
		HeapAdjust(L,1,i-1);//�Ի�����Ľṹ���¹���󶥶�
	}
	
}






