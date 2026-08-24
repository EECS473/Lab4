
/**
 * @file    h-bridge device driver
 * @author  Kevin Yang
 * @date    3/20/2017
 * @brief   Assignment code for a kernel module that controls a h-bridge on Raspberry Pi 4 or 5.
 *          Pin mounts and commands are defined below.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h> /* printk() */
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/err.h>

#include <linux/slab.h>  /* kmalloc() */
#include <linux/fs.h>    /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/types.h> /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h> /* O_ACCMODE */
#include <linux/ioport.h>
// #include <asm/system.h>   /* cli(), *_flags */
#include <linux/uaccess.h> /* copy_from/to_user */
#include <asm/io.h>

MODULE_LICENSE("Dual BSD/GPL");

/*
 * Input commands and pin mounts
 */

#define FORWARD 'F'
#define LEFT 'L'
#define BACK 'B'
#define RIGHT 'R'
#define STOP 'S'

// Adjust to add intermediate speeds
#define IDLE 0
int SPEED = 40;

// Adjust to reverse motor polarity
int LEFT_MOTOR = true;
int RIGHT_MOTOR = false;

// These pins are for the RPI4 B / RPI5, selected at runtime from Device Tree.

// PWM pins are configured in /boot/firmware/config.txt
// Left motor enable configured to pin 12
// Right motor enable configured to pin 13

// The Device Tree overlay maps:
//   "left"  -> PWM channel 0
//   "right" -> PWM channel 1
struct pwm_device *pwm0 = NULL; // pin 12
struct pwm_device *pwm1 = NULL; // pin 13

// Linux global GPIO numbering differs between the two Lab 4 images:
// Pi 4 (pinctrl-bcm2711): base 512
// Pi 5 (pinctrl-rp1):     base 569

// The GPIO base numbering will differ between the RPI4 and 5
// Refer to the initialization of the platform driver partd2_init(), which will
// define GPIO_BASE depending on the GPIO control chip
static int GPIO_BASE;

#define A_1 (GPIO_BASE + 5)  // Y1, left motor positive
#define A_2 (GPIO_BASE + 6)  // Y2, left motor negative
#define A_3 (GPIO_BASE + 19) // Y3, right motor positive
#define A_4 (GPIO_BASE + 26) // Y4, right motor negative

int memory_open(struct inode *inode, struct file *filp);
int memory_release(struct inode *inode, struct file *filp);
ssize_t memory_read(struct file *filp, char *buf, size_t count,
                    loff_t *f_pos);
ssize_t memory_write(struct file *filp, const char *buf, size_t count,
                     loff_t *f_pos);
void memory_exit(void);
int memory_init(void);
long memory_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

void setPin(int PIN);
void removePin(int PIN);
struct pwm_device *enPWM(int pwm_num);
void removePWM(struct pwm_device *pwm);
void moveRobot(char command);
void motorControl(bool ifLeftMotor, char command);
void pwm_duty_cycle(struct pwm_device *pwm, int percent);

// pointer to the Device Tree entry which is setup through the
// functions at the bottom
static struct device *partd2_dev;

struct file_operations memory_fops =
    {
        .read = memory_read,
        .write = memory_write,
        .open = memory_open,
        .release = memory_release,
        .unlocked_ioctl = memory_ioctl};

int memory_major = 60;
char *memory_buffer;

int memory_init(void)
{
    int result;
    result = register_chrdev(memory_major, "memory", &memory_fops);
    if (result < 0)
    {
        printk("Memory: cannot obtain major number %d\n", memory_major);
        return result;
    }

    /* Allocating memory for the buffer */
    memory_buffer = kmalloc(1, GFP_KERNEL);
    if (!memory_buffer)
    {
        result = -ENOMEM;
        goto fail;
    }

    memset(memory_buffer, 0, 1);
    printk("Inserting memory module\n");

    setPin(A_1);
    setPin(A_2);
    setPin(A_3);
    setPin(A_4);
    pwm0 = enPWM(0);

    // error checking if PWM is not found
    if (IS_ERR(pwm0))
    {
        result = PTR_ERR(pwm0);
        pwm0 = NULL;
        goto fail;
    }

    pwm1 = enPWM(1);

    // error checking if PWM is not found
    if (IS_ERR(pwm1))
    {
        result = PTR_ERR(pwm1);
        pwm1 = NULL;
        goto fail;
    }

    return 0;

fail:
    memory_exit();
    return result;
}

void memory_exit(void)
{
    unregister_chrdev(memory_major, "memory");
    if (memory_buffer)
    {
        kfree(memory_buffer);
        memory_buffer = NULL;
    }
    printk("Removing memory module\n");

    removePWM(pwm0);
    removePWM(pwm1);
    pwm0 = NULL;
    pwm1 = NULL;

    removePin(A_1);
    removePin(A_2);
    removePin(A_3);
    removePin(A_4);

    printk("GPIO freed, goodbye\n");
}

int memory_open(struct inode *inode, struct file *filp)
{
    return 0;
}

int memory_release(struct inode *inode, struct file *filp)
{
    return 0;
}

ssize_t memory_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    /* Transfering data to user space */
    /* Changing reading position as best suits */
    if (*f_pos == 0)
    {
        if (copy_to_user(buf, memory_buffer, 1))
            return count; // error
        *f_pos += 1;
        return 1;
    }
    else
    {
        return 0;
    }
}

ssize_t memory_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    int tmp = copy_from_user(memory_buffer, buf, 1);
    if (tmp != 0)
    {
        printk("mem_write error");
        return (count); // just do nothing but say you did all the chars
    }
    f_pos += 1;

    if (memory_buffer[0] == 'F')
    {
        moveRobot(FORWARD);
    }
    if (memory_buffer[0] == 'L')
    {
        moveRobot(LEFT);
    }
    if (memory_buffer[0] == 'B')
    {
        moveRobot(BACK);
    }
    if (memory_buffer[0] == 'R')
    {
        moveRobot(RIGHT);
    }
    if (memory_buffer[0] == 'S')
    {
        moveRobot(STOP);
    }

    return 1;
}

struct pwm_device *enPWM(int pwm_num)
{
    struct pwm_device *pwm;
    const char *pwm_name;

    // pulling left and right from device tree
    if (pwm_num == 0)
        pwm_name = "left";
    else if (pwm_num == 1)
        pwm_name = "right";
    else
    {
        printk("Invalid PWM number\n");
        return NULL;
    }

    pwm = devm_pwm_get(partd2_dev, pwm_name);

    // error checking if PWM is not found
    if (IS_ERR(pwm))
    {
        printk("Could not get PWM%d! error=%ld\n",
               pwm_num, PTR_ERR(pwm));
        return pwm;
    }

    // set PWM to 100hz with SPEED% duty cycle
    pwm_duty_cycle(pwm, SPEED);
    return pwm;
}

void removePWM(struct pwm_device *pwm)
{
    struct pwm_state state;

    if (!pwm)
        return;

    pwm_get_state(pwm, &state);
    state.enabled = false;
    pwm_apply_might_sleep(pwm, &state);

    // devm_pwm_get() automatically releases the PWM when the platform device is removed.
}

void setPin(int PIN)
{
    if (!gpio_is_valid(PIN))
    {
        printk("Invalid GPIO pin\n");
        return;
    }
    // Your stuff here.

    printk("GPIO pin %d exported... Pin state is currently: %d\n",
           PIN, gpio_get_value(PIN));
}

void removePin(int PIN)
{
    // Your stuff here.
}

void pwm_duty_cycle(struct pwm_device *pwm, int percent)
{
    struct pwm_state state;
    int result;

    // error checks
    if (!pwm)
        return;

    if (percent > 100)
    {
        printk("Invalid duty cycle\n");
        return;
    }

    pwm_get_state(pwm, &state);

    // PWM configuration step
    state.period = 10000000;
    state.duty_cycle = 100000ULL * percent;
    state.polarity = PWM_POLARITY_NORMAL;
    state.enabled = true;

    result = pwm_apply_might_sleep(pwm, &state);
    if (result)
        printk("Could not set PWM duty cycle: %d\n", result);
}

void moveRobot(char command)
{
    switch (command)
    {
    case FORWARD:
        motorControl(LEFT_MOTOR, FORWARD);
        motorControl(RIGHT_MOTOR, FORWARD);
        break;
    case LEFT:
        motorControl(LEFT_MOTOR, STOP);
        motorControl(RIGHT_MOTOR, FORWARD);
        break;
    case BACK:
        motorControl(LEFT_MOTOR, BACK);
        motorControl(RIGHT_MOTOR, BACK);
        break;
    case RIGHT:
        motorControl(LEFT_MOTOR, FORWARD);
        motorControl(RIGHT_MOTOR, STOP);
        break;
    case STOP:
        motorControl(LEFT_MOTOR, STOP);
        motorControl(RIGHT_MOTOR, STOP);
        break;
    default:
        printk("Illegal command input\n");
        break;
    }
}

void motorControl(bool ifLeftMotor, char command)
{
    struct pwm_device *enable = ifLeftMotor ? pwm0 : pwm1;
    int motorPos = ifLeftMotor ? A_1 : A_3;
    int motorNeg = ifLeftMotor ? A_2 : A_4;

    switch (command)
    {
    case FORWARD:
        pwm_duty_cycle(enable, SPEED);
        gpio_set_value(motorPos, 1);
        gpio_set_value(motorNeg, 0);
        break;
    case BACK:
        pwm_duty_cycle(enable, SPEED);
        gpio_set_value(motorPos, 0);
        gpio_set_value(motorNeg, 1);
        break;
    case STOP:
        gpio_set_value(motorPos, 0);
        gpio_set_value(motorNeg, 0);
        break;
    default:
        break;
    }
}

long memory_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    printk("<1>in ioctl\n");
    if (cmd == 0)
    {
        // your stuff here
    }
    else if (cmd == 1)
    { // adjust PWM
        if (arg <= 100)
        {
            SPEED = arg;
            pwm_duty_cycle(pwm0, SPEED);
            pwm_duty_cycle(pwm1, SPEED);
        }
        else
        {
            return -EINVAL;
        }
    }
    return (0); // success!
}

// For Debian Trixie, we need to add a platform driver
// to expose PWM to kernelspace. When the kernel is inserted,
// Linux will look for the Device Tree device and run the
// and initialize. Once the kernel is removed, the cleanup function partd2_remove
// will clean up the rest.

// platform driver initialization
// detect whether the board is RPI 4 or RPI 5 and assign correct GPIO_BASE
static int partd2_init(struct platform_device *pdev)
{
    if (of_machine_is_compatible("brcm,bcm2712"))
    {
        GPIO_BASE = 569;
        printk("Detected Raspberry Pi 5: GPIO base = %d\n", GPIO_BASE);
    }
    else if (of_machine_is_compatible("brcm,bcm2711"))
    {
        GPIO_BASE = 512;
        printk("Detected Raspberry Pi 4: GPIO base = %d\n", GPIO_BASE);
    }
    else
    {
        printk("Unsupported Raspberry Pi platform\n");
        return -ENODEV;
    }

    partd2_dev = &pdev->dev;
    return memory_init();
}

// cleanup function for the driver
static void partd2_remove(struct platform_device *pdev)
{
    memory_exit();
    partd2_dev = NULL;
}

// list Device Tree compatible strings supported by this driver
static const struct of_device_id partd2_of_match[] = {
    {.compatible = "eecs473,partd2-pwm"},
    {}};

// expose Device Tree match information
MODULE_DEVICE_TABLE(of, partd2_of_match);

// define the platform driver and Device Tree matching behavior
static struct platform_driver partd2_driver = {
    .probe = partd2_init,
    .remove = partd2_remove,
    .driver = {
        .name = "partd2-pwm",
        .of_match_table = partd2_of_match,
    },
};

// register partd2_driver when kernel is inserted and unregister when removed
module_platform_driver(partd2_driver);