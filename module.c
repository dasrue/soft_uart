// Original author : Adriano Marto Reis
// Original sourc  : https://github.com/adrianomarto/soft_uart
// Modified by     : Hippy

#include "raspberry_soft_uart.h"

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/version.h>
#include <linux/gpio.h>

#define SOFT_UART_MAJOR            0
#define N_PORTS                    1
#define NONE                       0
#define TX_BUFFER_FLUSH_TIMEOUT 4000  // milliseconds

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Original: Adriano Marto Reis / Modified: Hippy");
MODULE_DESCRIPTION("Software-UART for Raspberry Pi");
MODULE_VERSION("1.0");


static int gpio_tx = 22;
module_param(gpio_tx, int, 0);
MODULE_PARM_DESC(gpio_tx,"Transmit from Pi pin");
static int gpio_rx = 27;
module_param(gpio_rx, int, 0);
MODULE_PARM_DESC(gpio_rx,"Receive into Pi pin");
static int invert = 0;
module_param(invert, int, 0);
MODULE_PARM_DESC(invert,"Invert signal polarity");
static int invert_tx = 0;
module_param(invert_tx, int, 0);
MODULE_PARM_DESC(invert_tx,"Invert transmit signal polarity");
static int invert_rx = 0;
module_param(invert_rx, int, 0);
MODULE_PARM_DESC(invert_rx,"Invert receive signal polarity");
static int stop_bits = 1;
module_param(stop_bits, int, 0);
MODULE_PARM_DESC(stop_bits,"Number of stop bits on transmit");
static int break_tx = -1;
module_param(break_tx, int, 0);
MODULE_PARM_DESC(break_tx, "Transmit break sequence character");
static int break_rx = -1;
module_param(break_rx, int, 0);
MODULE_PARM_DESC(break_rx, "Receive break sequence character");
static char *device = "ttySOFT";
module_param(device, charp, 0000);
MODULE_PARM_DESC(device,"Device name");

// Module prototypes.
static int  soft_uart_open(struct tty_struct*, struct file*);
static void soft_uart_close(struct tty_struct*, struct file*);
static int  soft_uart_write(struct tty_struct*, const unsigned char*, int);
static int  soft_uart_write_room(struct tty_struct*);
static void soft_uart_flush_buffer(struct tty_struct*);
static int  soft_uart_chars_in_buffer(struct tty_struct*);
static void soft_uart_set_termios(struct tty_struct*, const struct ktermios*);
static void soft_uart_stop(struct tty_struct*);
static void soft_uart_start(struct tty_struct*);
static void soft_uart_hangup(struct tty_struct*);
static int  soft_uart_tiocmget(struct tty_struct*);
static int  soft_uart_tiocmset(struct tty_struct*, unsigned int, unsigned int);
static int  soft_uart_ioctl(struct tty_struct*, unsigned int, unsigned int long);
static void soft_uart_throttle(struct tty_struct*);
static void soft_uart_unthrottle(struct tty_struct*);

// Module operations.
static const struct tty_operations soft_uart_operations = {
  .open            = soft_uart_open,
  .close           = soft_uart_close,
  .write           = soft_uart_write,
  .write_room      = soft_uart_write_room,
  .flush_buffer    = soft_uart_flush_buffer,
  .chars_in_buffer = soft_uart_chars_in_buffer,
  .ioctl           = soft_uart_ioctl,
  .set_termios     = soft_uart_set_termios,
  .stop            = soft_uart_stop,
  .start           = soft_uart_start,
  .hangup          = soft_uart_hangup,
  .tiocmget        = soft_uart_tiocmget,
  .tiocmset        = soft_uart_tiocmset,
  .throttle        = soft_uart_throttle,
  .unthrottle      = soft_uart_unthrottle
};

// Driver instance.
static struct tty_driver* soft_uart_driver = NULL;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
static struct tty_port port;
#endif

/**
 * Module initialization.
 */
static int __init soft_uart_init(void)
{
  printk(KERN_INFO "soft_uart: Initializing module...\n");
  printk(KERN_INFO "soft_uart:   device     = /dev/%s0\n", device);
  if (gpio_tx < 0)
  { printk(KERN_INFO "soft_uart:   gpio_tx   = None\n"); }
  else
  { printk(KERN_INFO "soft_uart:   gpio_tx   = %d\n", gpio_tx); }
  if (gpio_rx < 0)
  { printk(KERN_INFO "soft_uart:   gpio_rx   = None\n"); }
  else
  { printk(KERN_INFO "soft_uart:   gpio_rx   = %d\n", gpio_rx); }
  printk(KERN_INFO "soft_uart:   invert    = %d\n", invert);
  printk(KERN_INFO "soft_uart:   invert_tx = %d\n", invert_tx);
  printk(KERN_INFO "soft_uart:   invert_rx = %d\n", invert_rx);
  printk(KERN_INFO "soft_uart:   stop_bits = %d\n", stop_bits);
  if (break_tx < 0)
  { printk(KERN_INFO "soft_uart:   break_tx  = None\n"); }
  else
  { printk(KERN_INFO "soft_uart:   break_tx  = 0x%02X\n", break_tx); }
  if (break_rx < 0)
  { printk(KERN_INFO "soft_uart:   break_rx  = None\n"); }
  else
  { printk(KERN_INFO "soft_uart:   break_rx  = 0x%02X\n", break_rx); }

  if (!raspberry_soft_uart_init(gpio_tx, gpio_rx, invert, invert_tx, invert_rx, stop_bits, break_tx, break_rx))
  {
    printk(KERN_ALERT "soft_uart: Failed initialize GPIO.\n");
    return -ENOMEM;
  }
    
  // Initializes the port.
  tty_port_init(&port);
  //port.low_latency = 0;

  // Allocates the driver.
  soft_uart_driver = tty_alloc_driver(N_PORTS, TTY_DRIVER_REAL_RAW);

  // Returns if the allocation fails.
  if (IS_ERR(soft_uart_driver))
  {
    printk(KERN_ALERT "soft_uart: Failed to allocate the driver.\n");
    return -ENOMEM;
  }

  // Initializes the driver.
  soft_uart_driver->owner                 = THIS_MODULE;
  soft_uart_driver->driver_name           = "soft_uart";
  soft_uart_driver->name                  = device;
  soft_uart_driver->major                 = SOFT_UART_MAJOR;
  soft_uart_driver->minor_start           = 0;
  soft_uart_driver->flags                 = TTY_DRIVER_REAL_RAW;
  soft_uart_driver->type                  = TTY_DRIVER_TYPE_SERIAL;
  soft_uart_driver->subtype               = SERIAL_TYPE_NORMAL;
  soft_uart_driver->init_termios          = tty_std_termios;
  soft_uart_driver->init_termios.c_ispeed = 4800;
  soft_uart_driver->init_termios.c_ospeed = 4800;
  soft_uart_driver->init_termios.c_cflag  = B4800 | CREAD | CS8 | CLOCAL;

  // Sets the callbacks for the driver.
  tty_set_operations(soft_uart_driver, &soft_uart_operations);

  // Link the port with the driver.
  tty_port_link_device(&port, soft_uart_driver, 0);

  // Registers the TTY driver.
  if (tty_register_driver(soft_uart_driver))
  {
    printk(KERN_ALERT "soft_uart: Failed to register the driver.\n");
    tty_driver_kref_put(soft_uart_driver);
    return -1; // return if registration fails
  }

  printk(KERN_INFO "soft_uart: Module initialized.\n");
  return 0;
}

/**
 * Cleanup function that gets called when the module is unloaded.
 */
static void __exit soft_uart_exit(void)
{
  printk(KERN_INFO "soft_uart: Finalizing the module...\n");
  
  // Finalizes the soft UART.
  if (!raspberry_soft_uart_finalize())
  {
    printk(KERN_ALERT "soft_uart: Something went wrong whilst finalizing the soft UART.\n");
  }
  
  // Unregisters the driver.
  tty_unregister_driver(soft_uart_driver);
  tty_driver_kref_put(soft_uart_driver);
  printk(KERN_INFO "soft_uart: Module finalized.\n");
}

/**
 * Opens a given TTY device.
 * @param tty given TTY device
 * @param file
 * @return error code.
 */
static int soft_uart_open(struct tty_struct* tty, struct file* file)
{
  int error = NONE;
    
  if (raspberry_soft_uart_open(tty))
  {
    printk(KERN_INFO "soft_uart: Device opened.\n");
  }
  else
  {
    printk(KERN_ALERT "soft_uart: Device busy.\n");
    error = -ENODEV;
  }
  
  return error;
}

/**
 * Closes a given TTY device.
 * @param tty
 * @param file
 */
static void soft_uart_close(struct tty_struct* tty, struct file* file)
{
  // Waits for the TX buffer to be empty before closing the UART.
  int wait_time = 0;
  while ((raspberry_soft_uart_get_tx_queue_size() > 0)
    && (wait_time < TX_BUFFER_FLUSH_TIMEOUT))
  {
    msleep(100);
    wait_time += 100;
  }
  
  if (raspberry_soft_uart_close())
  {
    printk(KERN_INFO "soft_uart: Device closed.\n");
  }
  else
  {
    printk(KERN_ALERT "soft_uart: Could not close the device.\n");
  }
}

/**
 * Writes the contents of a given buffer into a given TTY device.
 * @param tty given TTY device
 * @param buffer given buffer
 * @param buffer_size number of bytes contained in the given buffer
 * @return number of bytes successfuly written into the TTY device
 */
static int soft_uart_write(struct tty_struct* tty, const unsigned char* buffer, int buffer_size)
{
  return raspberry_soft_uart_send_string(buffer, buffer_size);
}

/**
 * Tells the kernel the number of bytes that can be written to a given TTY.
 * @param tty given TTY
 * @return number of bytes
 */
static int soft_uart_write_room(struct tty_struct* tty)
{
  return raspberry_soft_uart_get_tx_queue_room();
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_flush_buffer(struct tty_struct* tty)
{
}

/**
 * Tells the kernel the number of bytes contained in the buffer of a given TTY.
 * @param tty given TTY
 * @return number of bytes
 */
static int soft_uart_chars_in_buffer(struct tty_struct* tty)
{
  return raspberry_soft_uart_get_tx_queue_size();
}

/**
 * Sets the UART parameters for a given TTY (only the baudrate is taken into account).
 * @param tty given TTY
 * @param termios parameters
 */
static void soft_uart_set_termios(struct tty_struct* tty, const struct ktermios* termios)
{
  int cflag = 0;
  speed_t baudrate = tty_get_baud_rate(tty);
  printk(KERN_INFO "soft_uart: soft_uart_set_termios: baudrate = %d.\n", baudrate);

  // Gets the cflag.
  cflag = tty->termios.c_cflag;

  // Verifies the number of data bits (it must be 8).
  if ((cflag & CSIZE) != CS8)
  {
    printk(KERN_ALERT "soft_uart: Invalid number of data bits.\n");
  }
  
  // Verifies the number of stop bits (it must be 1).
  if (cflag & CSTOPB)
  {
    printk(KERN_ALERT "soft_uart: Invalid number of stop bits.\n");
  }
  
  // Verifies the parity (it must be none).
  if (cflag & PARENB)
  {
    printk(KERN_ALERT "soft_uart: Invalid parity.\n");
  }
  
  // Configure the baudrate.
  if (!raspberry_soft_uart_set_baudrate(baudrate))
  {
    printk(KERN_ALERT "soft_uart: Invalid baudrate.\n");
  }
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_stop(struct tty_struct* tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_stop.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_start(struct tty_struct* tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_start.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_hangup(struct tty_struct* tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_hangup.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static int soft_uart_tiocmget(struct tty_struct* tty)
{
  return 0;
}

/**
 * Does nothing.
 * @param tty
 * @param set
 * @param clear
 */
static int soft_uart_tiocmset(struct tty_struct* tty, unsigned int set, unsigned int clear)
{
  return 0;
}

/**
 * Does nothing.
 * @param tty
 * @param command
 * @param parameter
 */
static int soft_uart_ioctl(struct tty_struct* tty, unsigned int command, unsigned int long parameter)
{
  int error = NONE;

  switch (command)
  {
    case TIOCMSET:
      error = NONE;
      break;
 
    case TIOCMGET:
      error = NONE;
      break;
      
    default:
      error = -ENOIOCTLCMD;
      break;
  }

  return error;
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_throttle(struct tty_struct* tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_throttle.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_unthrottle(struct tty_struct* tty)
{
  printk(KERN_DEBUG "soft_uart: soft_uart_unthrottle.\n");
}

// Module entry points.
module_init(soft_uart_init);
module_exit(soft_uart_exit);
