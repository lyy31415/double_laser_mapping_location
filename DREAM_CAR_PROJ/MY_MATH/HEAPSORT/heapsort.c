#include "heapsort.h"

//用于交换数据
void swap(SqList *L,int i,int j)
{
	int temp=L->data[j];
	L->data[j]=L->data[i];
	L->data[i]=temp;
}


//把父节点和左右孩子节点中最大值和父节点互换，如果子节点值最大，
//则沿着最大的子节点一直比较并互换下去    注：此处均是针对大顶堆进行的操作
void HeapAdjust(SqList *L, int s, int m)
{
	int temp,j;
	temp=L->data[s];//把父节点的值暂存起来，以备后面节点值互换使用
	
	for(j=2*s;j<=m;j*=2)
	{
		if(j<m && L->data[j]<L->data[j+1])//找出左右子节点中最大值
		{
			++j;//j为最大值节点下标
		}
		
		if(temp>=L->data[j])
		{
			break;//如果父节点已经大于左右子节点，说明已经构造大顶堆成功，退出循环
		}
		
		L->data[s]=L->data[j];
		s=j;
	}
	
	L->data[s]=temp;//把父节点值放在此处，完成父节点和最大子节点的数据交互
}



//对结构体中数组元素进行堆排序
void HeapSort(SqList *L)
{
	int i;
	for(i=L->length/2;i>0;i--)//把L中元素构造成一个大顶堆
	{
		HeapAdjust(L,i,L->length);
	}
	
	for(i=L->length;i>1;i--)
	{
		swap(L,1,i);//把堆顶元素和最后一个元素互换
		HeapAdjust(L,1,i-1);//对互换后的结构重新构造大顶堆
	}
	
}






