#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>

/* LKM Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("William Keeling");
MODULE_DESCRIPTION("GPIO device to drive LED and read a switch");

/* Variables for device and device class */
static dev_t rc_device_num;
static struct class *gpio_class;
static struct cdev char_device;
extern unsigned long volatile jiffies;
unsigned long last_jiffies=0;
unsigned long sw_time;
unsigned long debounce; 
unsigned long long_time; 
int irq_num;
char sw_status = 0;
unsigned int on_cnt = 0;   // on time for LED 0-7
unsigned int off_cnt = 7;  // off time for LED 0-7
// lookup of abouve LED times to msecs
unsigned int time[8] = {0, 71, 142, 213, 284, 355, 426, 497}; 


static struct gpio_desc *rc_led = NULL;
static struct gpio_desc *rc_switch = NULL;
static struct task_struct *led_thread;

#define DRIVER_NAME "racecam_drv"
#define DRIVER_CLASS "GPIOClass"

/* read from user space*/
static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) 
{
	char out = (off_cnt << 5) | (on_cnt << 2) | sw_status;
	printk("RaceCam: read value %d\n", out);   //rm
	if (copy_to_user(user_buffer, &out, sizeof(out)) != 0) {
		printk("RaceCam: Copy to user space failed\n");
		return -1;
	}
	if (sw_status && sw_status != 3) sw_status = 0;
	return 0;
}
/* write from user space*/
static ssize_t driver_write(struct file *File, const char *user_buffer, size_t count, loff_t *offs) 
{
	char in;
	char switch_cmd;  // if no command ned to be pass to driver can be removed

	if (count != sizeof(in)) {
		printk("RaceCam: Invalid request length\n");  // error or woarn and take one one byte??
		return -1;
	}
	if (copy_from_user(&in, user_buffer, sizeof(in)) != 0) {
		printk("RaceCam: Copy from user space failed\n");
		return -1;
	}
	// switch_cmd removed -- off_cnt = in >> 2; on_cnt = off_cnt & 7; off_cnt >>= 3;
	switch_cmd = in & 3;
	in = in >> 2;
	on_cnt = in & 7;
	in = in >> 3;
	off_cnt = in;
	return sizeof(in);
} 
/* set device file permission for char device */
static char *gpio_devnode(const struct device *dev, umode_t *mode)
{
	if (mode) *mode = 0666;
	return NULL;
}
/* file operations structure for char device */
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = driver_read,
	.write = driver_write
};
/* thread function for LED control */
static int run_led(void *nothing)
{
	while(!kthread_should_stop())
	{
		if (on_cnt && !kthread_should_stop())
		{
			gpiod_set_value(rc_led, 1);
			msleep(time[on_cnt]);
		}
		if (off_cnt && !kthread_should_stop())
		{
			gpiod_set_value(rc_led, 0);
			msleep(time[off_cnt]);
		}
	}
	return 0;
} 
/* switch interupt handeler function */
static irqreturn_t switch_irq_handler(int irq, void *dev_id)
{
	printk("RaceCam: Interrupt was triggered and ISR was called!\n"); //rm
	unsigned long current_j = jiffies;
	unsigned long diff = current_j - last_jiffies;
	if (diff < debounce) 
	{
		printk("RaceCam: Interrupt was debounced!\n");  //rm
		return IRQ_HANDLED;
	}
	last_jiffies = current_j;
	int sws = gpiod_get_value(rc_switch);
	printk("RaceCam: get switch %d\n", sws); //rm
	if (!sws)  // active when switch open, inactive when switch closed/pressed
	{
		sw_time = current_j;
		sw_status = 3;  // 3 = currently pressed
	} 
	else
	{
		diff = current_j - sw_time;
		printk("RaceCam: %lu %lu %lu\n", sw_time, diff, long_time); //rm
		if (diff < long_time)
		{
			sw_status = 1; // 1 = completed short press
		} 
		else 
		{
			sw_status = 2; // 2 = completed long press
		}
		
	}
	printk("RaceCam: status %d\n", sw_status);  //rm
	return IRQ_HANDLED;
}
/* install platform driver fuction */ 
static int platform_probe(struct platform_device *pdev) 
{
	printk("RaceCam: Installing\n");
	
	struct device *dev = &pdev->dev;
	
	/* setup GPIOs for switch and led */
	if(!device_property_present(dev, "status-led-gpios")) 
	{
		printk("RaceCam: Error! Device property 'status-led-gpios' not found!\n");
		return -1;
	}
	if(!device_property_present(dev, "switch-gpios")) 
	{
		printk("RaceCam: Error! Device property 'switch-gpios' not found!\n");
		return -1;
	} 
	rc_led = gpiod_get(dev, "status-led", GPIOD_OUT_LOW); 
	if(IS_ERR(rc_led)) {
		printk("Racecam: Error! Could not setup the GPIO status_led\n");
		return -1 * IS_ERR(rc_led);
	}
	rc_switch = gpiod_get(dev, "switch", GPIOD_IN);
	if(IS_ERR(rc_switch)) {
		printk("Racecam: Error! Could not setup the GPIO switch\n");
		gpiod_put(rc_led);
		return -1 * IS_ERR(rc_switch);
	}
	
	/* setup status LED thread to enable blink rates */
	led_thread = kthread_create(run_led, NULL, "led_thread");
	if(led_thread != NULL)
	{
		wake_up_process(led_thread);
	}
	else {
		printk("Racecam: LED thread could not be created!\n");
		goto rm_gpio;
	}
	
	/* init switch irq */ 
	debounce = msecs_to_jiffies(12);  // debonce time
	long_time = msecs_to_jiffies(500); // long swtich press threashold 
	irq_num = gpiod_to_irq(rc_switch);
	if (irq_num < 0) 
	{ 
		printk("RaceCam: unable to get IRQ num %d\n", irq_num);
		goto rm_gpio;
	}
	if(request_irq(irq_num, switch_irq_handler, IRQF_TRIGGER_RISING | 
		IRQF_TRIGGER_FALLING, "switch_irq", NULL) != 0)
	{
		printk("RaceCam: Can not request interrupt nr.: %d\n", irq_num);
		goto rm_gpio;
	}

	/* create character device for userspace interface */
	if( alloc_chrdev_region(&rc_device_num, 0, 1, DRIVER_NAME) < 0) {
		printk("RaceCam driver: Device number not allocated!\n");
		goto rm_gpio;
	}
	printk("RaceCam driver: Device Major: %d, Minor: %d\n", 
		rc_device_num >> 20, rc_device_num && 0xfffff);

	if((gpio_class = class_create(DRIVER_CLASS)) == NULL) {
		printk("RaceCam driver: Class not created!\n");
		goto rm_dev;
	}
	gpio_class->devnode = gpio_devnode;

	if (device_create(gpio_class, NULL, rc_device_num, NULL, DRIVER_NAME) == NULL) {
		printk("RaceCam driver: Can not create device file!\n");
		goto rm_class;
	}

	cdev_init(&char_device, &fops);

	if(cdev_add(&char_device, rc_device_num, 1) == -1) {
		printk("RaceCam driver: Registering device to kernel failed!\n");
		goto unreg_dev;
	}

	printk("RaceCam driver: Installed\n");
	return 0;
rm_gpio:
  gpiod_put(rc_led);
  gpiod_put(rc_switch);
rm_dev:
	device_destroy(gpio_class, rc_device_num);
rm_class:
	class_destroy(gpio_class);
unreg_dev:
	unregister_chrdev_region(rc_device_num, 1);
	return -1; 
}
/* remove platform driver fuction */ 
static int platform_remove(struct platform_device *pdev) {
	kthread_stop(led_thread);
	gpiod_put(rc_led);
	gpiod_put(rc_switch);
	cdev_del(&char_device);
	device_destroy(gpio_class, rc_device_num);
	class_destroy(gpio_class);
	unregister_chrdev_region(rc_device_num, 1); 
	printk("RaceCam: Removed\n");
	return 0;
}

/* setup device tree/open firmware table for compatible matching 
 * to load this module when device tree overlay is initailized */
static struct of_device_id my_match_table[] = {
     {
             .compatible = "racecam",
     },
     {},
}; 
/* expose the above as a module alais so module is found and loaded */
MODULE_DEVICE_TABLE(of, my_match_table);

/* setup platform functions (probe and remove) and match table */
static struct platform_driver platform_driver = {
    .probe = platform_probe,
    .remove = platform_remove,
    .driver = {
        .name = "racecam_drv",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(my_match_table),
    },
}; 
/* expose platform struct so compatible matching can run fuctions at
 * platform device creation and destruction    */
module_platform_driver(platform_driver);


