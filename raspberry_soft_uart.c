            
#include "raspberry_soft_uart.h"
#include "queue.h"

#include <linux/gpio.h> 
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/div64.h>

static irqreturn_t handle_rx_start(int irq, void *device);
static enum hrtimer_restart handle_tx(struct hrtimer* timer);
static enum hrtimer_restart handle_rx(struct hrtimer* timer);
static void receive_character(unsigned char character);

static struct queue queue_tx;
static struct tty_struct* current_tty = NULL;
static DEFINE_MUTEX(current_tty_mutex);
static struct hrtimer timer_tx;
static struct hrtimer timer_rx;
static ktime_t period;
static ktime_t half_period;
static int gpio_tx = 0;
static int gpio_rx = 0;
static int rx_bit_index = -1;
static void (*rx_callback)(unsigned char) = NULL;
static int invert_tx = 0;
static int invert_rx = 0;
static int stop_bits = 0;
static int break_tx = -1;
static int break_rx = -1;
static int us = 0;

/**
 * Initializes the Raspberry Soft UART infrastructure.
 * This must be called during the module initialization.
 * The GPIO pin used as TX is configured as output.
 * The GPIO pin used as RX is configured as input.
 * @param gpio_tx GPIO pin used as TX
 * @param gpio_rx GPIO pin used as RX
 * @return 1 if the initialization is successful. 0 otherwise.
 */
int raspberry_soft_uart_init(const int _gpio_tx, const int _gpio_rx, const int _invert, const int _invert_tx, const int _invert_rx, const int _stop_bits, const int _break_tx, const int _break_rx)
{
  bool success = true;

  mutex_init(&current_tty_mutex);
  
  // Initializes the TX timer.
  hrtimer_init(&timer_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  timer_tx.function = &handle_tx;
  
  // Initializes the RX timer.
  hrtimer_init(&timer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  timer_rx.function = &handle_rx;
  
  // Initializes the GPIO pins.
  gpio_tx = _gpio_tx;
  gpio_rx = _gpio_rx;

  invert_tx = ( _invert_tx ^ _invert ) & 1;
  invert_rx = ( _invert_rx ^ _invert ) & 1;
  break_tx = _break_tx;
  break_rx = _break_rx;
  if ( _stop_bits <= 0 )
  {
    stop_bits = 1;
  }
  else
  {
    stop_bits = _stop_bits;
  }

  if (gpio_tx >= 0)
  {    
    success &= gpio_request(gpio_tx, "soft_uart_tx") == 0;
    success &= gpio_direction_output(gpio_tx, 1) == 0;
  }

  if (gpio_rx >= 0 )
  {
    success &= gpio_request(gpio_rx, "soft_uart_rx") == 0;
    success &= gpio_direction_input(gpio_rx) == 0;
  }

  // Initializes the interruption.
  if (gpio_rx >= 0)
  {
    if ( invert_rx == 0 )
    {
      success &= request_irq(
        gpio_to_irq(gpio_rx),
        (irq_handler_t) handle_rx_start,
        IRQF_TRIGGER_FALLING,
        "soft_uart_irq_handler",
        NULL) == 0;
      disable_irq(gpio_to_irq(gpio_rx));
    }
    else
    {
      success &= request_irq(
        gpio_to_irq(gpio_rx),
        (irq_handler_t) handle_rx_start,
        IRQF_TRIGGER_RISING,
        "soft_uart_irq_handler",
        NULL) == 0;
      disable_irq(gpio_to_irq(gpio_rx));
    };
    if (success && (gpio_tx >= 0) )
    {
      gpio_set_value(gpio_tx,1 ^ invert_tx);
    }
  }  
  return success;
}

/**
 * Finalizes the Raspberry Soft UART infrastructure.
 */
int raspberry_soft_uart_finalize(void)
{
  if (gpio_rx >= 0)
  {
    free_irq(gpio_to_irq(gpio_rx), NULL);
  }
  if (gpio_tx >= 0)
  {
    gpio_set_value(gpio_tx, 0);
    gpio_direction_input(gpio_rx);
    gpio_free(gpio_tx);
  }
  if (gpio_rx >= 0)
  {
    gpio_free(gpio_rx);
  }
  return 1;
}

/**
 * Opens the Soft UART.
 * @param tty
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_open(struct tty_struct* tty)
{
  int success = 0;
  mutex_lock(&current_tty_mutex);
  rx_bit_index = -1;
  if (current_tty == NULL)
  {
    current_tty = tty;
    if (gpio_tx >= 0)
    {
      initialize_queue(&queue_tx);
      queue_set_break_char(break_tx);
    }
    if (gpio_rx >= 0)
    {
      enable_irq(gpio_to_irq(gpio_rx));
    }
    success = 1;
  }
  mutex_unlock(&current_tty_mutex);
  return success;
}

/**
 * Closes the Soft UART.
 */
int raspberry_soft_uart_close(void)
{
  int success = 0;
  mutex_lock(&current_tty_mutex);
  if (current_tty != NULL)
  {
    if (gpio_rx >= 0)
    {
      disable_irq(gpio_to_irq(gpio_rx));
      hrtimer_cancel(&timer_rx);
    }
    if (gpio_tx >= 0)
    {
      hrtimer_cancel(&timer_tx);
    }
    current_tty = NULL;
    success = 1;
  }
  mutex_unlock(&current_tty_mutex);
  return success;
}

/**
 * Sets the Soft UART baudrate.
 * @param baudrate desired baudrate
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_set_baudrate(const int baudrate) 
{
  period = ktime_set(0, 1000000000/baudrate);
  half_period = ktime_set(0, 1000000000/baudrate/2);
  if (gpio_rx >= 0)
  {
    gpiod_set_debounce(gpio_to_desc(gpio_rx), 1000/baudrate/2);
  }
  if      (baudrate ==   9600) { us = 104; } // 104.16
  else if (baudrate ==  19200) { us =  52; } //  52.08
  else if (baudrate ==  38400) { us =  26; } //  26.04
  else if (baudrate ==  76800) { us =  13; } //  13.02
  else if (baudrate ==   7200) { us = 139; } // 138.88
  else if (baudrate ==  14400) { us =  69; } //  69.44
  else if (baudrate ==  28800) { us =  35; } //  34.72
  else if (baudrate ==  57600) { us =  17; } //  17.36
  else if (baudrate == 115200) { us =   8; } //   8.68
  else                         { us =   0; }
  printk(KERN_INFO "soft_uart:   period = %llu, us = %d\n", period,us);
  return 1;
}

/**
 * Adds a given string to the TX queue.
 * @paran string given string
 * @param string_size size of the given string
 * @return The amount of characters successfully added to the queue.
 */
int raspberry_soft_uart_send_string(const unsigned char* string, int string_size)
{
  int result = 1;
  if (gpio_tx  >= 0)
  {
    result = enqueue_string(&queue_tx, string, string_size);
  
    // Starts the TX timer if it is not already running.
    if (!hrtimer_active(&timer_tx))
    {
      hrtimer_start(&timer_tx, period, HRTIMER_MODE_REL);
    }
  }

  return result;
}

/*
 * Gets the number of characters that can be added to the TX queue.
 * @return number of characters.
 */
int raspberry_soft_uart_get_tx_queue_room(void)
{
  return get_queue_room(&queue_tx);
}

/*
 * Gets the number of characters in the TX queue.
 * @return number of characters.
 */
int raspberry_soft_uart_get_tx_queue_size(void)
{
  return get_queue_size(&queue_tx);
}

/**
 * Sets the callback function to be called on received character.
 * @param callback the callback function
 */
int raspberry_soft_uart_set_rx_callback(void (*callback)(unsigned char))
{
	rx_callback = callback;
	return 1;
}

//-----------------------------------------------------------------------------
// Internals
//-----------------------------------------------------------------------------

/**
 * If we are waiting for the RX start bit, then starts the RX timer. Otherwise,
 * does nothing.
 */
static irqreturn_t handle_rx_start(int irq, void *device)
{
  if (rx_bit_index == -1)
  {
    hrtimer_start(&timer_rx, half_period, HRTIMER_MODE_REL);
    disable_irq_nosync(gpio_to_irq(gpio_rx)); 
  }
  return IRQ_HANDLED;
}


/**
 * Dequeues a character from the TX queue and sends it.
 */
static enum hrtimer_restart handle_tx(struct hrtimer* timer)
{
  static int character = 0;
  static int bit_count = 0;

  // Start bit.
  if (bit_count == 0)
  {
    if (dequeue_character(&queue_tx, &character))
    {
      if (character == SET_BREAK_VAL) // -2 0xFFFE - Set Break (lsb=0)
      {
        gpio_set_value(gpio_tx, invert_tx);
        hrtimer_forward(&timer_tx, ktime_get(), period);
        bit_count = 9 + stop_bits;
        return HRTIMER_RESTART;
      }
      else if (character == CLR_BREAK_VAL) // -1 0xFFFF - Clear Break (lsb=1)
      {
        gpio_set_value(gpio_tx, 1 ^ invert_tx);
        hrtimer_forward(&timer_tx, ktime_get(), period);
        bit_count = 8 + stop_bits;
        return HRTIMER_RESTART;
      }
      else if (us != 0) // If fast baud then don't swap out
      {
        gpio_set_value(gpio_tx, invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 0 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 1 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 2 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 3 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 4 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 5 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 6 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, ( ( character >> 7 ) & 1 ) ^ invert_tx); udelay(us);
        gpio_set_value(gpio_tx, 1 ^ invert_tx);
        character = 0xFF;
        bit_count = stop_bits-1;
        hrtimer_forward(&timer_tx, ktime_get(), period);
        return HRTIMER_RESTART;
      }
      else
      {
        gpio_set_value(gpio_tx, invert_tx);
        hrtimer_forward(&timer_tx, ktime_get(), period);
        bit_count = 8 + stop_bits;
        return HRTIMER_RESTART;
      }
    }
  }
  
  // Data bits plus Stop bits
  else
  {
    gpio_set_value(gpio_tx, ( character & 1 ) ^ invert_tx);
    hrtimer_forward(&timer_tx, ktime_get(), period);
    if (character >= 0)
    {
      character = ( character >> 1 ) | 0x80;
    }
    bit_count--;
    return HRTIMER_RESTART;  
  }

  return HRTIMER_NORESTART;
}

/*
 * Receives a character and sends it to the kernel.
 */
static enum hrtimer_restart handle_rx(struct hrtimer* timer)
{
  static int character = 0;
  ktime_t captured_time = ktime_get();
  int rx_bit = gpio_get_value(gpio_rx) ^ invert_rx;
  if (rx_bit_index < 0)
  {
    // Start bit : rx_bit_index=-1
    if (rx_bit == 0)
    {
      // Start bit as expected
      hrtimer_forward(&timer_rx, captured_time, period);
      rx_bit_index = 8;
      return HRTIMER_RESTART;
    }
    else
    {
      // False start bit - Ignore
      // See : https://github.com/adrianomarto/soft_uart/issues/4
      enable_irq(gpio_to_irq(gpio_rx));
      return HRTIMER_NORESTART;
    }
  }
  else if (rx_bit_index > 0)
  {
    // Data bits : rx_bit_index=8..1
    hrtimer_forward(&timer_rx, captured_time, period);
    character = ( character >> 1 ) | ( rx_bit << 7 );
    rx_bit_index--;
    return HRTIMER_RESTART;
  }
  else
  {
    // Stop Bit : rx_bit_index = 0
    if (rx_bit == 0)
    {
      // Break / Framing error
      if ( (break_rx>=0) && (character==0x00) )
      {
        receive_character(break_rx);
        receive_character(0x01);
      }
      else
      {
        // Hippy - Or should we ignore framing errors ?
        receive_character(character);
      }
    }
    else
    {
      // Stop bit correct - Received character
      if (character == break_rx)
      {
        receive_character(break_rx);
        receive_character(0x00);
      }
      else
      {
        receive_character(character);
      }
    }
    rx_bit_index = -1;
    // See : https://github.com/adrianomarto/soft_uart/issues/4
    enable_irq(gpio_to_irq(gpio_rx));
    return HRTIMER_NORESTART;
  }
}

/**
 * Adds a given (received) character to the RX buffer, which is managed by the kernel,
 * and then flushes (flip) it.
 * @param character given character
 */
void receive_character(unsigned char character)
{
  mutex_lock(&current_tty_mutex);
  if (rx_callback != NULL) {
	  (*rx_callback)(character);
  } else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    if (current_tty != NULL && current_tty->port != NULL)
    {
      tty_insert_flip_char(current_tty->port, character, TTY_NORMAL);
      tty_flip_buffer_push(current_tty->port);
    }
#else
    if (tty != NULL)
    {
      tty_insert_flip_char(current_tty, character, TTY_NORMAL);
      tty_flip_buffer_push(tty);
    }
#endif
  }
  mutex_unlock(&current_tty_mutex);
}
