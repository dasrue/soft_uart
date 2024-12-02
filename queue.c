#include "queue.h"

static int break_char    = -1;
static int break_pending = 0;
static int break_active  = 0;

void queue_set_break_char(int _break_char)
{
  break_char    = _break_char;
  break_pending = 0;
  break_active  = 0;
}

/**
 * Initializes a given queue.
 * @param queue given queue
 */
void initialize_queue(struct queue* queue)
{
  queue->size  = 0;
  queue->front = 0;
  queue->rear  = 0;
}

/**
 * Adds a given character into a given queue.
 * @param queue given queue
 * @param character given character
 * @return 1 if the character is added to the queue. 0 if the queue is full.
 */
int enqueue_character(struct queue* queue, int character)
{
  int success = 0;

  if (break_char >= 0)
  {
    if ( character >= 0 )
    {
      if (break_pending != 0)
      {
        if (character == 1)
        {
          character = SET_BREAK_VAL;
          break_active = 1;
        }
        else if (character == 2)
        {
          character = CLR_BREAK_VAL;
          break_active = 0;
        }
        else
        {
          if (break_active != 0) { enqueue_character(queue, CLR_BREAK_VAL); }
          character = break_char;
        }
        break_pending = 0;
      }
      else if (character == break_char)
      {
        break_pending = 1;
        return 1;
      }
      else if (break_active != 0) { enqueue_character(queue, CLR_BREAK_VAL); }
    }
  }

  if (queue->size < QUEUE_MAX_SIZE)
  {
    if (queue->size != 0)
    {
      queue->rear++;
      if (queue->rear >= QUEUE_MAX_SIZE)
      {
        queue->rear = 0;
      }
    }
    else
    {
      queue->rear = 0;
      queue->front = 0;
    }
    queue->data[queue->rear] = character;
    queue->size++;
    success = 1;
  }
  return success;
}

/**
 * Gets a character from a fiven queue.
 * @param queue given queue
 * @param character a character
 * @return 1 if a character is fetched from the queue. 0 if the queue is empy.
 */
int dequeue_character(struct queue* queue, int* character)
{
  int success = 0;
  if (queue->size > 0)
  {
    *character = queue->data[queue->front];
    queue->front++;
    if (queue->front >= QUEUE_MAX_SIZE)
    {
      queue->front = 0;
    }
    queue->size--;
    success = 1;
  }
  return success;
}

/**
 * Adds a given string to a given queue.
 * @param queue given queue
 * @param string given string
 * @param string_size size of the given string
 * @return The amount of characters successfully added to the queue.
 */
int enqueue_string(struct queue* queue, const unsigned char* string, int string_size)
{
  int n = 0;
  while (n < string_size && enqueue_character(queue, string[n]))
  {
    n++;
  }
  return n;
}

/**
 * Gets the number of characters that can be added to a given queue.
 * @return number of characters.
 */
int get_queue_room(struct queue* queue)
{
  return QUEUE_MAX_SIZE - queue->size;
}

/**
 * Gets the number of characters contained in a given queue.
 * @return number of characters.
 */
int get_queue_size(struct queue* queue)
{
  return queue->size;
}

