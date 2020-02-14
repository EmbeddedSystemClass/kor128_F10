#include "main.h"
#include "ring_buffer.h"


void InitQueue(Queue *queue)
{
    queue->front = queue->rear = 0;							// front�� rear�� 0���� ����
}

int IsFull(Queue *queue)									// ���� ť���� �� á���� ������� üũ�� �� �ְ� rear ���� ������ �� ���¸� �����մϴ�.
{
    
    return NEXT(queue->rear) == queue->front;				// ���� rear�� front�� ������ �� �� ����
}

int IsEmpty(Queue *queue)
{
    return queue->front == queue->rear;						// front�� rear�� ������ �� ����
}

void Enqueue(Queue *queue, uint8_t data)
{
	uint8_t dummy;
    if (IsFull(queue))										// ť�� �� á�� ��
    {
        //printf("queue full\r\n");
				dummy = Dequeue(queue);
        //return;
    }
    queue->buf[queue->rear] = data;							// rear �ε����� ������ ����
    queue->rear = NEXT(queue->rear);						// rear�� ���� ��ġ�� ����
}

uint8_t Dequeue(Queue *queue)
{
    uint8_t re = 0;
    if (IsEmpty(queue))										// ť�� ����� ��
    {
        //printf("queue empty\r\n");
        return re;
    }
    re = queue->buf[queue->front];							// front �ε����� ������ ���� re�� ����
    queue->front = NEXT(queue->front);						// front�� ���� ��ġ�� ����
    return re;
}

uint16_t Len_queue(Queue *queue)
{
	return ((QUEUE_SIZE - queue->front + queue->rear)%QUEUE_SIZE);
}

// queue Test
void print_queue (Queue *queue) 
{
	uint16_t i;
	//printf ("Queue From Front------> To Rear \r\n");
	printf("f:%1d,r:%1d ",queue->front,queue->rear);
	for(i = queue->front; i != queue->rear; i = ((i+1)%QUEUE_SIZE))
	{
		printf("[%d]%2d :", i, queue->buf[i]);
	}
	printf (" L[%2d]\r\n",Len_queue(queue));
}

