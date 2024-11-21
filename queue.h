#ifndef QUEUE_H
#define QUEUE_H

#define QUEUE_MAX_SIZE  256

struct queue
{
  int front;
  int rear;
  int size;
  int data[QUEUE_MAX_SIZE];
};


#define SET_BREAK_VAL -2
#define CLR_BREAK_VAL -1

void queue_set_break_char(int _break_char);
void initialize_queue(struct queue* queue);
int  enqueue_character(struct queue* queue, int character);
int  dequeue_character(struct queue* queue, int* character);
int  enqueue_string(struct queue* queue, const unsigned char* string, int string_size);
int  get_queue_room(struct queue* queue);
int  get_queue_size(struct queue* queue);

#endif
