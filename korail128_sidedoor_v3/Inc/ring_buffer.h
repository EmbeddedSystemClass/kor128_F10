#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include "stdint.h"

#define QUEUE_SIZE  2000						// ���� ť ũ��� QUEUE_SIZE-1 �̴�.
#define NEXT(index)   ((index+1)%QUEUE_SIZE)	// ���� ť���� �ε����� �����ϴ� ��ũ�� �Լ�
 
typedef struct Queue							// Queue ����ü ����
{
    uint8_t buf[QUEUE_SIZE];					// �����
    uint16_t front;								// ���� �ε���(���� �������� ������ �����Ͱ� �ִ� �ε���)
    uint16_t rear;								// ������ �ε���
}Queue;
 


#ifdef __cplusplus
 extern "C" {
#endif

void InitQueue(Queue *queue);					// ť �ʱ�ȭ
int IsFull(Queue *queue);						// ť�� �� á���� Ȯ��
int IsEmpty(Queue *queue);						// ť�� ������� Ȯ��
void Enqueue(Queue *queue, uint8_t data);		// ť�� ����
uint8_t Dequeue(Queue *queue);					// ť���� ����
void print_queue (Queue *queue);
uint16_t Len_queue(Queue *queue);


#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H__ */
