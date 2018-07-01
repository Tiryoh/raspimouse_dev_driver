/*
 *
 * rtcounter.c
 * device driver of Raspberry Pi Gibbon
 *
 * Copyright (C) 2017 Tiryoh <tiryoh@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("Daisuke Sato");
MODULE_DESCRIPTION("i2c device driver of Raspberry Pi Mouse");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static struct cdev *cdev_array = NULL;
static struct class *class_cntr = NULL;
static struct class *class_cntl = NULL;
static struct class *class_cnts = NULL;

static volatile uint32_t *gpio_base = NULL;

static volatile int cdev_index = 0;
static volatile int open_counter = 0;

#define MAX_BUFLEN 64

#define DEV_MAJOR 0
#define DEV_MINOR 0

#define RPI_GPIO_P2MASK (uint32_t)0xffffffff

#define RPI_REG_BASE 0x3f000000

#define RPI_GPIO_OFFSET 0x200000
#define RPI_GPIO_SIZE 0xC0
#define RPI_GPIO_BASE (RPI_REG_BASE + RPI_GPIO_OFFSET)
#define REG_GPIO_NAME "RaspberryPi GPIO"

#define RPI_GPF_INPUT 0x00
#define RPI_GPF_OUTPUT 0x01

#define RPI_GPFSEL0_INDEX 0
#define RPI_GPFSEL1_INDEX 1
#define RPI_GPFSEL2_INDEX 2
#define RPI_GPFSEL3_INDEX 3

#define RPI_GPSET0_INDEX 7
#define RPI_GPCLR0_INDEX 10

#define GPIO_PULLNONE 0x0
#define GPIO_PULLDOWN 0x1
#define GPIO_PULLUP 0x2

#define NUM_DEV_CNTR 1
#define NUM_DEV_CNTL 1
#define NUM_DEV_CNTS 1
#define NUM_DEV_TOTAL NUM_DEV_CNTR + NUM_DEV_CNTL + NUM_DEV_CNTS

#define DEV_ADDR_CNTR 0x10
#define DEV_ADDR_CNTL 0x11

#define DEVNAME_CNTR "rtcounter_r"
#define DEVNAME_CNTL "rtcounter_l"
#define DEVNAME_CNTS "rtcounter"

#define I2C_RETRY_MAX 3

#define CNT_ADDR_MSB 0x10
#define CNT_ADDR_LSB 0x11

static int _major_cntr = DEV_MAJOR;
static int _minor_cntr = DEV_MINOR;
static int _major_cntl = DEV_MAJOR;
static int _minor_cntl = DEV_MINOR;
static int _major_cnts = DEV_MAJOR;
static int _minor_cnts = DEV_MINOR;

struct i2c_counter_device {
    struct i2c_client client;
};

static struct i2c_counter_device *i2c_clients[2] = {NULL};
/* struct i2c_counter_device *rtcounters[2]; */
/* static struct i2c_counter_device *tmp1 = NULL; */
/* static struct i2c_counter_device *tmp2 = NULL; */
/* static volatile struct i2c_counter_device *tmp1 = NULL; */
/* static volatile struct i2c_counter_device *tmp2 = NULL; */

static int i2c_counter_set(struct i2c_client *client, int setval)
{
    int ret = 0;
    int lsb = 0, msb = 0;

    printk(KERN_INFO "set 0x%x = 0x%x\n", client->addr, setval);
    msb = (setval >> 8) & 0xFF;
    lsb = setval & 0xFF;
    /* printk(KERN_INFO "set 0x%x msb = 0x%x\n", client->addr, msb); */
    usleep_range(27,100);
    ret = i2c_smbus_write_byte_data(client, 0x10, msb);
    if( ret < 0 ) {
        printk(KERN_ERR "%s: Could not write to i2c counter device, addr=0x%x\n", __func__, client->addr);
        return -ENODEV;
    }
    usleep_range(27,100);
    /* printk(KERN_INFO "set 0x%x lsb = 0x%x\n", client->addr, lsb); */
    ret = i2c_smbus_write_byte_data(client, 0x11, lsb);
    if( ret < 0 ) {
        printk(KERN_ERR "%s: Could not write to i2c counter device, addr=0x%x\n", __func__, client->addr);
        return -ENODEV;
    }
    usleep_range(27,100);
    return ret;
}

static int i2c_counter_read(struct i2c_client *client, int *ret)
{
    int lsb = 0, msb = 0;

    printk(KERN_INFO "read 0x%x\n", client->addr);

    usleep_range(27,100);

    lsb = i2c_smbus_read_byte_data(client, 0x11);
    if( lsb < 0 ) {
        printk(KERN_ERR "%s: Could not read from i2c counter device, addr=0x%x\n", __func__, client->addr);
        return -ENODEV;
    }
    usleep_range(27,100);
    msb = i2c_smbus_read_byte_data(client, 0x10);
    if( msb < 0 ) {
        printk(KERN_ERR "%s: Could not read from i2c counter device, addr=0x%x\n", __func__, client->addr);
        return -ENODEV;
    }
    usleep_range(27,100);

    *ret = ((msb << 8) & 0xFF00) + (lsb & 0xFF);

    printk(KERN_INFO "0x%x == 0x%x\n", client->addr, *ret);

    return 0;
}

static int i2c_counter_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    /* struct i2c_counter_device *dev; */
    /* int ret; */
    /*  */
    /* printk( KERN_INFO "probing i2c device, addr=0x%x\n", client->addr); */
    /*  */
    /* if (!i2c_check_functionality(client->adapter, */
    /*             I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) { */
    /*     printk(KERN_ERR "%s: needed i2c functionality is not supported\n", __func__); */
    /*     return -ENODEV; */
    /* } */
    /*  */
    /* dev = kzalloc(sizeof(struct i2c_counter_device), GFP_KERNEL); */
    /* if (dev == NULL) { */
    /*     printk(KERN_ERR "%s: no memory\n", __func__); */
    /*     return -ENOMEM; */
    /* } */
    /*  */
    /* dev->client = client; */
    /* i2c_set_clientdata(client, dev); */

    return 0;
}

static int i2c_counter_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    int ret = 0;
    /* struct i2c_counter_device *dev; */

    printk(KERN_INFO "detect i2c device, addr=0x%x\n", client->addr);
    printk(KERN_INFO "i2c adding ... \n" );
    i2c_clients[client->addr - 0x10]->client = *client;

    /* if(client->addr == 0x10){ */
    /*     tmp1 = (struct i2c_counter_device *)kmalloc(sizeof(struct i2c_counter_device), GFP_KERNEL); */
    /*     if (tmp1 == NULL) { */
    /*         printk(KERN_ERR "%s: no memory\n", __func__); */
    /*         return -ENOMEM; */
    /*     } */
    /*     tmp1->client = *client; */
    /*     i2c_set_clientdata(client, &tmp1); */
    /* } */
    /* if(client->addr == 0x11){ */
    /*     tmp2 = (struct i2c_counter_device *)kmalloc(sizeof(struct i2c_counter_device), GFP_KERNEL); */
    /*     if (tmp2 == NULL) { */
    /*         printk(KERN_ERR "%s: no memory\n", __func__); */
    /*         return -ENOMEM; */
    /*     } */
    /*     tmp2->client = *client; */
    /*     i2c_set_clientdata(client, &tmp2); */
    /* } */
    printk(KERN_INFO "addr=0x%x\n", client->addr);
    printk(KERN_INFO "i2c_clients[%x] addr=0x%x\n",(client->addr - 0x10), i2c_clients[client->addr - 0x10]->client.addr);
    /* if(client->addr == 0x10)printk(KERN_INFO "tmp1 addr=0x%x\n",tmp1->client.addr); */
    /* if(client->addr == 0x11)printk(KERN_INFO "tmp2 addr=0x%x\n",tmp2->client.addr); */

    /* dev = (struct i2c_counter_device *)i2c_get_clientdata(client); */
    /* dev = kzalloc(sizeof(struct i2c_counter_device), GFP_KERNEL); */
    printk(KERN_INFO "i2c tmp saved\n" );
    /* dev = kzalloc(sizeof((struct i2c_counter_device *)i2c_get_clientdata(client)), GFP_KERNEL); */
    /* if (dev == NULL) { */
    /*     printk(KERN_ERR "%s: no memory\n", __func__); */
    /*     return -ENOMEM; */
    /* } */
    printk(KERN_INFO "i2c added ... \n" );

    ret = i2c_counter_set(client, 0);

    // int hoge;
    // i2c_counter_read(client, &hoge);

    return ret;
}

static int i2c_counter_remove(struct i2c_client *client)
{
    /* struct i2c_counter_device *dev; */

    printk( KERN_INFO "i2c removing ... \n" );
    /* dev = (struct i2c_counter_device *)i2c_get_clientdata(client); */
    /* kfree(dev); */
    /* printk( KERN_INFO "i2c removed\n" ); */
    return 0;
}

static struct i2c_device_id i2c_counter_id[] = {
    { "rtcntr", 0  },
    { "rtcntl", 1  },
    {  },
};

MODULE_DEVICE_TABLE(i2c, i2c_counter_id);

static const unsigned short i2c_counter_addr[] = { 
    DEV_ADDR_CNTR,
    DEV_ADDR_CNTL,
    I2C_CLIENT_END
};

static struct i2c_driver i2c_counter_driver = {  
    .driver = {
        .name = "rtcounter",
        .owner = THIS_MODULE,
    },
    .id_table = i2c_counter_id,
    /* .probe = i2c_counter_probe, */
    .remove = i2c_counter_remove,
    .class = I2C_CLASS_DDC | I2C_CLASS_SPD,
    .detect = i2c_counter_detect,
    .address_list = i2c_counter_addr,
};

static int rpi_gpio_function_set(int pin, uint32_t func) {
  int index = RPI_GPFSEL0_INDEX + pin / 10;
  uint32_t shift = (pin % 10) * 3;
  uint32_t mask = ~(0x07 << shift);
  gpio_base[index] = (gpio_base[index] & mask) | ((func & 0x07) << shift);
  return 1;
}

static void rpi_gpio_set32(uint32_t mask, uint32_t val) {
  gpio_base[RPI_GPSET0_INDEX] = val & mask;
}

static void rpi_gpio_clear32(uint32_t mask, uint32_t val) {
  gpio_base[RPI_GPCLR0_INDEX] = val & mask;
}

static int parse_count(const char __user *buf, size_t count, int *ret)
{
    char cval;
    int error = 0, i = 0, tmp, bufcnt = 0, freq;
    size_t readcount = count;
    int sgn = 1;

    char *newbuf = kmalloc(sizeof(char)*count, GFP_KERNEL);

    while(readcount > 0)
    {
        if(copy_from_user(&cval, buf + i, sizeof(char)))
        {
            kfree(newbuf);
            return -EFAULT;
        }

        if(cval == '-')
        {
            if(bufcnt == 0)
            {
                sgn = -1;
            }
        }
        else if(cval < '0' || cval > '9')
        {
            newbuf[bufcnt] = 'e';
            error = 1;
        }
        else
        {
            newbuf[bufcnt] = cval;
        }

        i++;
        bufcnt++;
        readcount--;

        if(cval == '\n')
        {
            break;
        }
    }

    freq = 0;
    for(i = 0, tmp = 1; i < bufcnt; i++)
    {
        char c = newbuf[bufcnt - i - 1];

        if( c >= '0' && c <= '9')
        {
            freq += ( newbuf[bufcnt - i - 1]  -  '0' ) * tmp;
            tmp *= 10;
        }
    }

    *ret = sgn * freq; 

    kfree(newbuf);

    return bufcnt;
}


/*
 *  cntr_write - Set value to right pulse counter
 *  Write function of /dev/rtcounter_r
 */
static ssize_t cntr_write(struct file *filep, const char *buf, size_t count,
                         loff_t *pos) {
    int bufcnt = 0;
    /* int minor = *((int *)filep->private_data); */
    int cntr_count = 0;

    if (count < 0) return 0;

    bufcnt = parse_count(buf, count, &cntr_count);

    i2c_counter_set(&(i2c_clients[1]->client), cntr_count);

    printk(KERN_INFO "Set CNTR:%d\n", cntr_count);
    return bufcnt;
}


/*
 *  cntr_read - Read value from right pulse counter
 *  Read function of /dev/rtcounter_r
 */
static ssize_t cntr_read(struct file *filep, char __user *buf, size_t count,
                               loff_t *f_pos) {
    unsigned char rw_buf[MAX_BUFLEN];
    int buflen = 0;
    unsigned int ret = 0;
    int cntr_count = 0;

    if(*f_pos > 0) return 0; /* End of file */

    /*
     * get sensor data
     */
    /* printk(KERN_INFO "going to read 0x%x\n", tmp2->client.addr); */
    /* i2c_counter_read(&(tmp2->client), &cntr_count); */
    printk(KERN_INFO "going to read 0x%x\n", i2c_clients[1]->client.addr);
    i2c_counter_read(&(i2c_clients[1]->client), &cntr_count);

    /*
     * set sensor data to rw_buf(static buffer)
     */

    snprintf(rw_buf, sizeof(rw_buf), "%d\n", cntr_count);
    buflen = strlen(rw_buf);
    count = buflen;

    /*
     * copy data to user area
     */
    if(copy_to_user((void *)buf, &rw_buf, count))
    {
        printk(KERN_INFO "err read buffer from ret  %d\n", ret);
        printk(KERN_INFO "err read buffer from %s\n", rw_buf);
        printk(KERN_INFO "err sample_char_read size(%d)\n", count);
        printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
        return -EFAULT;
    }
    *f_pos += count;

    return count;
}

static ssize_t cntl_write(struct file *filep, const char *buf, size_t count,
        loff_t *pos) {
    int bufcnt = 0;
    /* int minor = *((int *)filep->private_data); */
    int cntl_count;

    if (count < 0) return 0;

    bufcnt = parse_count(buf, count, &cntl_count);

    i2c_counter_set(&(i2c_clients[0]->client), cntl_count);

    printk(KERN_INFO "SET CNTL:%d\n", cntl_count);
    return bufcnt;
}

static ssize_t cntl_read(struct file *filep, char __user *buf, size_t count,
                               loff_t *f_pos) {
    unsigned char rw_buf[MAX_BUFLEN];
    int buflen = 0;
    unsigned int ret = 0;
    int rf;

    if(*f_pos > 0) return 0; /* End of file */

    /*
     * get sensor data
     */
    ret = 0;
    /* printk(KERN_INFO "going to read 0x%x\n", tmp2->client.addr); */
    /* i2c_counter_read(&(tmp2->client), &rf); */
    printk(KERN_INFO "going to read 0x%x\n", i2c_clients[0]->client.addr);
    i2c_counter_read(&(i2c_clients[0]->client), &rf);

    /*
     * set sensor data to rw_buf(static buffer)
     */
    snprintf(rw_buf, sizeof(rw_buf), "%d\n", rf);
    buflen = strlen(rw_buf);
    count = buflen;

    /*
     * copy data to user area
     */
    if(copy_to_user((void *)buf, &rw_buf, count))
    {
        printk(KERN_INFO "err read buffer from ret  %d\n", ret);
        printk(KERN_INFO "err read buffer from %s\n", rw_buf);
        printk(KERN_INFO "err sample_char_read size(%d)\n", count);
        printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
        return -EFAULT;
    }
    *f_pos += count;

    return count;
}

static ssize_t cnts_write(struct file *filep, const char *buf, size_t count,
        loff_t *pos) {
    int cntr_count,cntl_count;
    char *newbuf = kmalloc(sizeof(char) * count, GFP_KERNEL);

    if (count < 0) return 0;
    if (copy_from_user(newbuf, buf, sizeof(char) * count)) {
        kfree(newbuf);
        return -EFAULT;
    }
    sscanf(newbuf, "%d%d\n", &cntl_count, &cntl_count);
    kfree(newbuf);

    i2c_counter_set(&(i2c_clients[0]->client), cntl_count);
    i2c_counter_set(&(i2c_clients[1]->client), cntr_count);

    printk(KERN_INFO "SET CNTL:%d\n", cntl_count);
    printk(KERN_INFO "SET CNTR:%d\n", cntl_count);
    return count;
}

static ssize_t cnts_read(struct file *filep, char __user *buf, size_t count,
                               loff_t *f_pos) {
    unsigned char rw_buf[MAX_BUFLEN];
    int buflen = 0;
    unsigned int ret = 0;
    int cntr_count = 0, cntl_count = 0;

    if(*f_pos > 0) return 0; /* End of file */

    /*
     * get sensor data
     */
    printk(KERN_INFO "going to read 0x%x\n", i2c_clients[0]->client.addr);
    i2c_counter_read(&(i2c_clients[0]->client), &cntl_count);
    printk(KERN_INFO "going to read 0x%x\n", i2c_clients[1]->client.addr);
    i2c_counter_read(&(i2c_clients[1]->client), &cntr_count);

    /*
     * set sensor data to rw_buf(static buffer)
     */
    snprintf(rw_buf, sizeof(rw_buf), "%d %d\n", cntl_count, cntr_count);
    buflen = strlen(rw_buf);
    count = buflen;

    /*
     * copy data to user area
     */
    if(copy_to_user((void *)buf, &rw_buf, count))
    {
        printk(KERN_INFO "err read buffer from %s\n", rw_buf);
        printk(KERN_INFO "err sample_char_read size(%d)\n", count);
        printk(KERN_INFO "sample_char_read size err(%d)\n", -EFAULT);
        return -EFAULT;
    }
    *f_pos += count;
    buflen=0;

    return count;
}

static int dev_open(struct inode *inode, struct file *filep) {
  /* int retval; */
  int *minor = (int *)kmalloc(sizeof(int), GFP_KERNEL);
  int major = MAJOR(inode->i_rdev);
  *minor = MINOR(inode->i_rdev);

  printk(KERN_INFO "device open request, major:%d minor: %d \n", major, *minor);

  filep->private_data = (void *)minor;

  /* retval = led_gpio_map(); */
  /* if (retval != 0) { */
  /*   printk(KERN_ERR "gpio_map failed.\n"); */
  /*   return retval; */
  /* } */
  open_counter++;
  return 0;
}

static int dev_release(struct inode *inode, struct file *filep) {
  kfree(filep->private_data);
  open_counter--;
  if (open_counter <= 0) {
    iounmap(gpio_base);
    gpio_base = NULL;
  }
  return 0;
}

static struct file_operations cntr_fops = {
    .open = dev_open,
    .release = dev_release,
    .write = cntr_write,
    .read = cntr_read,
};

static struct file_operations cntl_fops = {
    .open = dev_open,
    .release = dev_release,
    .write = cntl_write,
    .read = cntl_read,
};

static struct file_operations cnts_fops = {
    .open = dev_open,
    .release = dev_release,
    .write = cnts_write,
    .read = cnts_read,
};

static int cntr_register_dev(void) {
  int retval;
  dev_t dev;
  dev_t devno;
  int i;

  retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_CNTR, DEVNAME_CNTR);

  if (retval < 0) {
    printk(KERN_ERR "alloc_chrdev_region failed.\n");
    return retval;
  }
  _major_cntr = MAJOR(dev);

  class_cntr = class_create(THIS_MODULE, DEVNAME_CNTR);
  if (IS_ERR(class_cntr)) {
    return PTR_ERR(class_cntr);
  }

  for (i = 0; i < NUM_DEV_CNTR; i++) {
    devno = MKDEV(_major_cntr, _minor_cntr + i);

    cdev_init(&(cdev_array[cdev_index]), &cntr_fops);
    cdev_array[cdev_index].owner = THIS_MODULE;
    if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
      printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_cntr + i);
    } else {
      device_create(class_cntr, NULL, devno, NULL, DEVNAME_CNTR "%u",
                    _minor_cntr + i);
    }
    cdev_index++;
  }

  return 0;
}

static int cntl_register_dev(void) {
  int retval;
  dev_t dev;
  dev_t devno;
  int i;

  retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_CNTL, DEVNAME_CNTL);

  if (retval < 0) {
    printk(KERN_ERR "alloc_chrdev_region failed.\n");
    return retval;
  }
  _major_cntl = MAJOR(dev);

  class_cntl = class_create(THIS_MODULE, DEVNAME_CNTL);
  if (IS_ERR(class_cntl)) {
    return PTR_ERR(class_cntl);
  }

  for (i = 0; i < NUM_DEV_CNTL; i++) {
    devno = MKDEV(_major_cntl, _minor_cntl + i);

    cdev_init(&(cdev_array[cdev_index]), &cntl_fops);
    cdev_array[cdev_index].owner = THIS_MODULE;
    if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
      printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_cntl + i);
    } else {
      device_create(class_cntl, NULL, devno, NULL, DEVNAME_CNTL "%u",
                    _minor_cntl + i);
    }
    cdev_index++;
  }

  return 0;
}

static int cnts_register_dev(void) {
  int retval;
  dev_t dev;
  dev_t devno;
  int i;

  retval = alloc_chrdev_region(&dev, DEV_MINOR, NUM_DEV_CNTS, DEVNAME_CNTS);

  if (retval < 0) {
    printk(KERN_ERR "alloc_chrdev_region failed.\n");
    return retval;
  }
  _major_cnts = MAJOR(dev);

  class_cnts = class_create(THIS_MODULE, DEVNAME_CNTS);
  if (IS_ERR(class_cnts)) {
    return PTR_ERR(class_cnts);
  }

  for (i = 0; i < NUM_DEV_CNTS; i++) {
    devno = MKDEV(_major_cnts, _minor_cnts + i);

    cdev_init(&(cdev_array[cdev_index]), &cnts_fops);
    cdev_array[cdev_index].owner = THIS_MODULE;
    if (cdev_add(&(cdev_array[cdev_index]), devno, 1) < 0) {
      printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_cnts + i);
    } else {
      device_create(class_cnts, NULL, devno, NULL, DEVNAME_CNTS "%u",
                    _minor_cnts + i);
    }
    cdev_index++;
  }

  return 0;
}

static int __init init_mod(void) {
    int retval;
    size_t size;
    /* struct i2c_adapter *adapter; */

    printk(KERN_INFO "rtmouse device driver initializing...\n");

    size = sizeof(struct cdev) * NUM_DEV_TOTAL;
    cdev_array = (struct cdev *)kmalloc(size, GFP_KERNEL);

    printk(KERN_INFO "loading %s...\n", DEVNAME_CNTR);
    retval = cntr_register_dev();
    if (retval != 0) {
        printk(KERN_ALERT "rtcntr driver register failed.\n");
        return retval;
    }
    printk(KERN_INFO "loading %s...\n", DEVNAME_CNTL);
    retval = cntl_register_dev();
    if (retval != 0) {
        printk(KERN_ALERT "rtcntl driver register failed.\n");
        return retval;
    }
    printk(KERN_INFO "loading %s...\n", DEVNAME_CNTS);
    retval = cnts_register_dev();
    if (retval != 0) {
        printk(KERN_ALERT "rtcnts driver register failed.\n");
        return retval;
    }

    printk(KERN_INFO "%d devices loaded.\n", NUM_DEV_TOTAL);

    printk(KERN_INFO "prepareing i2c memory...\n");
    /* i2c_dev_array = (struct i2c_client *)kzalloc(sizeof(struct i2c_client) * 2, GFP_KERNEL); */
    /* if (i2c_dev_array == NULL) { */
    /*     printk(KERN_ERR "%s: no memory\n", __func__); */
    /*     return -ENOMEM; */
    /* } */

    i2c_clients[0] = (struct i2c_counter_device *)kzalloc(sizeof(struct i2c_counter_device), GFP_KERNEL);
    if (i2c_clients[0] == NULL) {
        printk(KERN_ERR "%s: no memory\n", __func__);
        return -ENOMEM;
    }
    i2c_clients[1] = (struct i2c_counter_device *)kzalloc(sizeof(struct i2c_counter_device), GFP_KERNEL);
    if (i2c_clients[1] == NULL) {
        printk(KERN_ERR "%s: no memory\n", __func__);
        return -ENOMEM;
    }

    /* tmp1 = (struct i2c_counter_device *)kzalloc(sizeof(struct i2c_counter_device), GFP_KERNEL); */
    /* if (tmp1 == NULL) { */
    /*     printk(KERN_ERR "%s: no memory\n", __func__); */
    /*     return -ENOMEM; */
    /* } */
    /* tmp2 = (struct i2c_counter_device *)kzalloc(sizeof(struct i2c_counter_device), GFP_KERNEL); */
    /* if (tmp2 == NULL) { */
    /*     printk(KERN_ERR "%s: no memory\n", __func__); */
    /*     return -ENOMEM; */
    /* } */

    /* i2c_clients[0]->addr = 0x10; */
    /* i2c_clients[1]->addr = 0x11; */
    /*  */
    /* printk(KERN_INFO "i2c_clients[%x] addr=0x%x\n", 0, i2c_clients[0]->addr); */
    /* printk(KERN_INFO "i2c_clients[%x] addr=0x%x\n", 1, i2c_clients[1]->addr); */
    /* printk(KERN_INFO "tmp1 addr=0x%x\n",tmp1->addr); */
    /* printk(KERN_INFO "tmp2 addr=0x%x\n",tmp2->addr); */

    printk(KERN_INFO "loading i2c devices...\n");
    retval = i2c_add_driver(&i2c_counter_driver);
    if (retval != 0) {
        printk(KERN_ALERT "i2c_driver add error.\n");
        return retval;
    }
    //retval = i2c_register_board_info(i2c-1, i2c_counter_info, ARRAY_SIZE(i2c_counter_info));

    /* adapter = i2c_get_adapter(0); */
    /* if (adapter == NULL) { */
    /*     printk(KERN_ALERT "i2c_board add error.\n"); */
    /*     return -ENXIO; */
    /* } */

    /* tsc2007_client = i2c_new_device(adapter, &raspi_board_info); */
    /*  */
    /* if (retval != 0) { */
    /*     printk(KERN_ALERT "i2c_board add error.\n"); */
    /*     return retval; */
    /* } */
    printk(KERN_INFO "i2c pulse counter driver initialized.\n");
    /* printk(KERN_INFO "i2c_clients[%x] addr=0x%x\n", 0, i2c_clients[0]->addr); */
    /* printk(KERN_INFO "i2c_clients[%x] addr=0x%x\n", 1, i2c_clients[1]->addr); */
    /* printk(KERN_INFO "tmp1 addr=0x%x\n",tmp1->client.addr); */
    /* printk(KERN_INFO "tmp2 addr=0x%x\n",tmp2->client.addr); */

    return 0;
}

static void __exit cleanup_mod(void) {
    int i;
    dev_t devno;
    dev_t devno_top;

    for (i = 0; i < NUM_DEV_TOTAL; i++) {
        cdev_del(&(cdev_array[i]));
    }

    // rtcntr
    devno_top = MKDEV(_major_cntr, _minor_cntr);
    for (i = 0; i < NUM_DEV_CNTR; i++) {
        devno = MKDEV(_major_cntr, _minor_cntr + i);
        device_destroy(class_cntr, devno);
    }
    unregister_chrdev_region(devno_top, NUM_DEV_CNTR);

    // rtcntl
    devno_top = MKDEV(_major_cntl, _minor_cntl);
    for (i = 0; i < NUM_DEV_CNTL; i++) {
        devno = MKDEV(_major_cntl, _minor_cntl + i);
        device_destroy(class_cntl, devno);
    }
    unregister_chrdev_region(devno_top, NUM_DEV_CNTL);

    // rtcnts
    devno_top = MKDEV(_major_cnts, _minor_cnts);
    for (i = 0; i < NUM_DEV_CNTS; i++) {
        devno = MKDEV(_major_cnts, _minor_cnts + i);
        device_destroy(class_cnts, devno);
    }
    unregister_chrdev_region(devno_top, NUM_DEV_CNTS);

    //remove device nodes
    class_destroy(class_cntr);
    class_destroy(class_cntl);
    class_destroy(class_cnts);

    kfree(cdev_array);

    //I2C module
    i2c_del_driver(&i2c_counter_driver);

    /* kfree(i2c_dev_array); */
    /* kfree(tmp1); */
    /* kfree(tmp2); */
    kfree(i2c_clients[0]);
    kfree(i2c_clients[1]);

    printk("module being removed at %lu\n", jiffies);
}

module_init(init_mod);
module_exit(cleanup_mod);
