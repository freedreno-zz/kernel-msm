/*
 
 */
#include <asm/atomic.h>
#include <asm/ioctls.h>

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

#include <linux/gpio.h>
#include <mach/clk.h>


#include <linux/mfd/pm8xxx/pm8921.h>

#include "msm8960_spi_fpga.h"

#define SPI1_DRIVER_NAME  "spi1_dev"
#define SPI8_DRIVER_NAME  "spi8_dev"
#define SPI10_DRIVER_NAME  "spi10_dev"

#define PM8921_GPIO_BASE                  NR_GPIO_IRQS
#define PM8921_IRQ_BASE                   (NR_MSM_IRQS + NR_GPIO_IRQS)
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)    (pm_gpio - 1 + PM8921_GPIO_BASE)

static struct spi_device *spi1_client;
static struct spi_device *spi8_client;
static struct spi_device *spi10_client;

struct mutex spi1_lock;
struct mutex spi8_lock;
struct mutex spi10_lock;

struct clk *fpga_clk;

unsigned char dummy_buf[DUMMY_BUF_SIZE];

/* may change depend on customer requirment */
#define READ_FPGA_DATA_CMD    0xAA
unsigned char read_cmd[1];


static int get_gpio_from_id(int id, int *type)
{
   int i;
   int gpio;
   
   /* BANK 2 CPU GPIO */
   if (id > 200)
   {
      for(i=0; i<FPGA_BANK2_CPU_PIO_COUNT; i++)
      {
         if (fpga_bank2_cpu_pio[i].id == id)
         {
            gpio = fpga_bank2_cpu_pio[i].gpio;
            *type = FPGA_CPU_PIO;
            return gpio;
         }
      }
      /* nopt found */
      return -1;
   }
   /* BANK1 PM GPIO */
   else if (id == 124 || id == 136 || id == 137 || id == 140 || id == 141)
   {
      for(i=0; i<FPGA_BANK1_PM_PIO_COUNT; i++)
      {
         if (fpga_bank1_pm_pio[i].id == id)
         {
            gpio = fpga_bank1_pm_pio[i].pm_gpio;
            *type = FPGA_PM_PIO;
            return gpio;
         }
      }
      /* nopt found */
      return -1;
   }
   /* BANK1 CPU GPIO */
   else
   {
      for(i=0; i<FPGA_BANK1_CPU_PIO_COUNT; i++)
      {
         if (fpga_bank1_cpu_pio[i].id == id)
         {
            gpio = fpga_bank1_cpu_pio[i].gpio;
            *type = FPGA_CPU_PIO;
            return gpio;
         }
      }
      /* nopt found */
      return -1;
   }
   
   return -1;
}

static int fpga_spi_read(struct spi_device *spi_client, unsigned char *data, int size)
{
   struct spi_message  m;
   struct spi_transfer t;

   if (!spi_client) 
   {
      pr_err("%s spi_client is NULL\n", __func__);
      return -EINVAL;
   }

   memset(&t, 0, sizeof t);
   read_cmd[0] = READ_FPGA_DATA_CMD;
   t.tx_buf = read_cmd;
   t.rx_buf = data;
   t.len = size;
   spi_setup(spi_client);
   spi_message_init(&m);
   spi_message_add_tail(&t, &m);

   if (spi_sync(spi_client, &m))
      pr_err("%s: SPI read failed\n", __func__);

   return m.status;
}

static int fpga_spi_write(struct spi_device *spi_client, unsigned char *data, int size)
{
   struct spi_message  m;
   struct spi_transfer t;

   if (!spi_client) 
   {
      pr_err("%s spi_client is NULL\n", __func__);
      return -EINVAL;;
   }

   memset(&t, 0, sizeof t);
   t.tx_buf = data;
   t.len = size;
   spi_setup(spi_client);
   spi_message_init(&m);
   spi_message_add_tail(&t, &m);

   if (spi_sync(spi_client, &m))
      pr_err("%s: SPI write failed\n", __func__);
 
   return m.status;
}

static int fpga_spi_exchange(struct spi_device *spi_client, unsigned char *tx_data, char *rx_data, int size)
{
   struct spi_message  m;
   struct spi_transfer t;

   if (!spi_client)
   {
      pr_err("%s spi_client is NULL\n", __func__);
      return -EINVAL;
   }

   memset(&t, 0, sizeof t);
   t.tx_buf = tx_data;
   t.rx_buf = rx_data;
   t.len = size;
   spi_setup(spi_client);
   spi_message_init(&m);
   spi_message_add_tail(&t, &m);

   if (spi_sync(spi_client, &m))
      printk("%s: SPI exchange failed\n", __func__);

   return m.status;
}


static ssize_t fpga_image_download(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
   int delay = 0;
   int rc;
   void *fpga_image; 
   
   if (!spi8_client) 
   {
      pr_err("%s spi8_client is NULL\n", __func__);
      return -EINVAL;
   }
   
   mutex_lock(&spi8_lock);
   
   fpga_image = kzalloc(count, GFP_KERNEL);
   if (!fpga_image)
   {
       pr_err("%s: memory allocation error\n", __func__);
       mutex_unlock(&spi8_lock);
       return -ENOMEM;
   }
   memcpy(fpga_image, buf, count);
   
   
   gpio_set_value(FPGA_RESET_GPIO, 0);
   gpio_set_value(SPI8_SS, 0);
   usleep(10);
   gpio_set_value(FPGA_RESET_GPIO, 1);
   
   msleep(10);

   rc = fpga_spi_write(spi8_client, (unsigned char *)fpga_image, count);
   if (rc < 0)
   {
       pr_err("%s: FPGA write error\n", __func__);
       kfree(fpga_image);
       mutex_unlock(&spi8_lock);
       return rc;
   }

   while (gpio_get_value(FPGA_CDONE_GPIO) != 1)
   {
      msleep(1);
      delay++;
      if (delay > 1000)
      {
         pr_err("%s: FPGA write timeout!\n", __func__);
         kfree(fpga_image);
         mutex_unlock(&spi8_lock);
         return -1;
      }
   }

   rc = fpga_spi_write(spi8_client, dummy_buf, DUMMY_BUF_SIZE);
   if (rc < 0)
   {
       pr_err("%s: FPGA write error\n", __func__);
       kfree(fpga_image);
       mutex_unlock(&spi8_lock);
       return rc;
   }
   
   gpio_set_value(SPI8_SS, 1);
   pr_info("writing FPGA image done\n");
   
   kfree(fpga_image);
   mutex_unlock(&spi8_lock);
   return 0;
}
   
static long fpga_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
   int rc = 0;
   int type;
   struct pm_gpio param;
   
   pr_info("%s: %d\n", __func__, cmd);
   
   switch(cmd)
   {
   case FPGA_IOCTL_PIO_SET_DIRCTION:
      {
         int gpio;
         FPGA_IOCTL_PIO_t pio;
         
         pr_debug("FPGA_IOCTL_PIO_SET_DIRCTIONT\n");
         if (copy_from_user(&pio, (void*)arg, sizeof(pio)))
         {
            rc = -EFAULT;
            break;
         }
         gpio = get_gpio_from_id(pio.id, &type);
         if (gpio < 0)
         {
            rc = -EFAULT;
            break;
         }
         switch(type)
         {
         case FPGA_CPU_PIO:
            if (pio.val == FPGA_PIO_INPUT)
               rc = gpio_direction_input(gpio);
            else
               rc = gpio_direction_output(gpio, 0);
            break;
         case FPGA_PM_PIO:
            if (pio.val == FPGA_PIO_INPUT)
            {
               param.direction = PM_GPIO_DIR_IN;
               param.pull = PM_GPIO_PULL_NO;
               param.function = PM_GPIO_FUNC_NORMAL;
               param.vin_sel = PM_GPIO_VIN_S4;  //1.8v
               param.inv_int_pol = 0;
               rc = pm8xxx_gpio_config(gpio, &param);
            }
            else
            {
               param.direction = PM_GPIO_DIR_OUT;
               param.pull = PM_GPIO_PULL_NO;
               param.out_strength   = PM_GPIO_STRENGTH_MED;
               param.function = PM_GPIO_FUNC_NORMAL;
               param.vin_sel = PM_GPIO_VIN_S4;
               param.output_buffer  = PM_GPIO_OUT_BUF_CMOS;
               param.output_value = 0;
               rc = pm8xxx_gpio_config(gpio, &param);
            }
            break;
         }
         break;
      }
   case FPGA_IOCTL_PIO_READ:
      {
         int gpio;
         FPGA_IOCTL_PIO_t pio;
         
         pr_debug("FPGA_IOCTL_PIO_READ\n");
         if (copy_from_user(&pio, (void*)arg, sizeof(pio))) 
         {
            rc = -EFAULT;
            break;
         }
         gpio = get_gpio_from_id(pio.id, &type);
         if (gpio < 0)
         {
            rc = -EFAULT;
            break;
         }
         pio.val = gpio_get_value(gpio);
         if (copy_to_user((void*)arg, &pio, sizeof(pio))) 
         {
            rc = -EFAULT;
            break;
         }
         break;
      }
    case FPGA_IOCTL_PIO_WRITE:
      {
         int gpio;
         FPGA_IOCTL_PIO_t pio;
         
         pr_debug("FPGA_IOCTL_PIO_WRITE\n");
         if (copy_from_user(&pio, (void*)arg, sizeof(pio))) 
         {
            rc = -EFAULT;
            break;
         }
         gpio = get_gpio_from_id(pio.id, &type);
         if (gpio < 0)
         {
            rc = -EFAULT;
            break;
         }
         gpio_set_value(gpio, pio.val);
         break;
      }
    case FPGA_IOCTL_SPI_READ:
      {
         FPGA_IOCTL_SPI_t spi_data;
         void *spi_buf; 
         
         pr_debug("FPGA_IOCTL_SPI_READ\n");
         if (copy_from_user(&spi_data, (void*)arg, sizeof(spi_data))) 
         {
            rc = -EFAULT;
            break;
         }   
         switch(spi_data.spi)
         {
         case 1:
            if (!spi1_client) 
            {
               pr_err("%s spi1_client is NULL\n", __func__);
               rc = -EINVAL;
               break;
            }
            mutex_lock(&spi1_lock);
            gpio_set_value(SPI1_CS, 0);
            
            spi_buf = kzalloc(spi_data.size, GFP_KERNEL);
            if (!spi_buf)
            {
               pr_err("%s: memory allocation error\n", __func__);
               mutex_unlock(&spi1_lock);
               return -ENOMEM;
            }
            
            rc = fpga_spi_read(spi1_client, (unsigned char *)spi_buf, spi_data.size);
            if (rc < 0)
            {
               pr_err("%s: FPGA spi1 read error\n", __func__);
               kfree(spi_buf);
               gpio_set_value(SPI1_CS, 1);
               mutex_unlock(&spi1_lock);
               return rc;
            }
            memcpy(spi_data.buff, spi_buf, spi_data.size);
         
            kfree(spi_buf);      
            gpio_set_value(SPI1_CS, 1);
            mutex_unlock(&spi1_lock);
            break;
         case 10:
            if (!spi10_client) 
            {
               pr_err("%s spi10_client is NULL\n", __func__);
               rc = -EINVAL;
               break;
            }
            mutex_lock(&spi10_lock);
            gpio_set_value(SPI10_CS, 0);
            
            spi_buf = kzalloc(spi_data.size, GFP_KERNEL);
            if (!spi_buf)
            {
               pr_err("%s: memory allocation error\n", __func__);
               mutex_unlock(&spi10_lock);
               return -ENOMEM;
            }
            
            rc = fpga_spi_read(spi10_client, (unsigned char *)spi_buf, spi_data.size);
            if (rc < 0)
            {
               pr_err("%s: FPGA spi10 read error\n", __func__);
               kfree(spi_buf);
               gpio_set_value(SPI10_CS, 1);
               mutex_unlock(&spi10_lock);
               return rc;
            }
            
            memcpy(spi_data.buff, spi_buf, spi_data.size);
         
            kfree(spi_buf);
            gpio_set_value(SPI10_CS, 1);
            mutex_unlock(&spi10_lock);
            break;
         default:
            pr_err("Not support spi%d\n", spi_data.spi);
         }
         if (copy_to_user((void*)arg, &spi_data, sizeof(spi_data)))
         {
            rc = -EFAULT;
            break;
         }
         break;
      }
    case FPGA_IOCTL_SPI_WRITE:
      {
         FPGA_IOCTL_SPI_t spi_data;
         void *spi_buf, *rx_spi_buf; 
            
         pr_debug("FPGA_IOCTL_SPI_WRITE\n");
         if (copy_from_user(&spi_data, (void*)arg, sizeof(spi_data))) 
         {
            rc = -EFAULT;
            break;
         }
         switch(spi_data.spi)
         {
         case 1:
            if (!spi1_client) 
            {
               pr_err("%s spi1_client is NULL\n", __func__);
               rc = -EINVAL;
               break;
            }
            mutex_lock(&spi1_lock);
            gpio_set_value(SPI1_CS, 0);
            
            spi_buf = kzalloc(spi_data.size, GFP_KERNEL);
            if (!spi_buf)
            {
               pr_err("%s: memory allocation error\n", __func__);
               mutex_unlock(&spi1_lock);
               return -ENOMEM;
            }
	    rx_spi_buf = kzalloc(spi_data.size, GFP_KERNEL);
	    
	    if (!rx_spi_buf)
            {
               pr_err("%s: rx_spi_buf allocation error\n", __func__);
               kfree(spi_buf);
               gpio_set_value(SPI1_CS, 1);
               mutex_unlock(&spi1_lock);
                return -ENOMEM;
            } 
	    memcpy(spi_buf, spi_data.buff, spi_data.size);
            
            rc = fpga_spi_exchange(spi1_client, (unsigned char *)spi_buf, (unsigned char *)rx_spi_buf, spi_data.size);
            if (rc < 0)
            {
               pr_err("%s: FPGA spi1 write error\n", __func__);
               kfree(spi_buf);
               kfree(rx_spi_buf);
               gpio_set_value(SPI1_CS, 1);
               mutex_unlock(&spi1_lock);
               return rc;
            }

            memcpy(spi_data.buff, rx_spi_buf, spi_data.size);	

            if (copy_to_user((void*)arg, &spi_data, sizeof(spi_data)))
            {
               pr_err("%s: FPGA spi1 copy_to_user error\n", __func__);
               rc = -EFAULT;
            }

            kfree(spi_buf);
            kfree(rx_spi_buf);
            gpio_set_value(SPI1_CS, 1);
            mutex_unlock(&spi1_lock);
            break;
         case 10:
            if (!spi10_client) 
            {
               pr_err("%s spi10_client is NULL\n", __func__);
               rc = -EINVAL;
               break;
            }
            mutex_lock(&spi10_lock);
            gpio_set_value(SPI10_CS, 0);
            
            spi_buf = kzalloc(spi_data.size, GFP_KERNEL);
            if (!spi_buf)
            {
               pr_err("%s: memory allocation error\n", __func__);
               mutex_unlock(&spi10_lock);
               return -ENOMEM;
            }
            
	    rx_spi_buf = kzalloc(spi_data.size, GFP_KERNEL);
            if (!rx_spi_buf)
            {
               pr_err("%s: memory allocation error\n", __func__);
               kfree(spi_buf);
               gpio_set_value(SPI10_CS, 1);
               mutex_unlock(&spi1_lock);
               return -ENOMEM;
            }

             memcpy(spi_buf, spi_data.buff, spi_data.size);
 
            rc = fpga_spi_exchange(spi1_client, (unsigned char *)spi_buf, (unsigned char *)rx_spi_buf, spi_data.size);
            if (rc < 0)
            {
               pr_err("%s: FPGA spi10 write error\n", __func__);
               kfree(spi_buf);
               kfree(rx_spi_buf);
               gpio_set_value(SPI10_CS, 1);
               mutex_unlock(&spi10_lock);
               return rc;
            }

            memcpy(spi_data.buff, rx_spi_buf, spi_data.size);

            if (copy_to_user((void*)arg, &spi_data, sizeof(spi_data)))
            {
               pr_err("%s: FPGA spi10 copy_to_user error\n", __func__);
               rc = -EFAULT;
            }

            kfree(spi_buf);
            
            kfree(rx_spi_buf);
            gpio_set_value(SPI10_CS, 1);
            mutex_unlock(&spi10_lock);
            break;
         default:
            pr_err("Not support spi%d\n", spi_data.spi);
         }
         break;
      }
    default:
      pr_err("Not support FPGA_IOCTL function %d\n", cmd);
      break;
   }
   
   return rc;
}

static int __devinit spi1_probe(struct spi_device *spi)
{
   int rc;
   
   pr_info("%s: %s\n", __func__, spi->modalias);
   
   spi1_client = spi;
   
   mutex_init(&spi1_lock);
   
   rc = gpio_request(SPI1_CS, "spi1_cs");
   if (rc) 
   {
      pr_err("Request SPI1_CS failed, rc=%d\n", rc);
      return -ENODEV;
   }
   rc = gpio_direction_output(SPI1_CS, 0);
   if (rc) 
   {
      pr_err("Unable to set SPI1_CS direction\n");
      goto free_gpio;
   }
   return 0;
free_gpio:
   gpio_free(SPI1_CS);
   return -ENODEV;
}

static int __devexit spi1_remove(struct spi_device *spi)
{
   spi1_client = NULL;
   
   gpio_free(SPI1_CS);
   return 0;
}

static int __devinit spi8_probe(struct spi_device *spi)
{
   int rc, i;
   
   pr_info("%s: %s\n", __func__, spi->modalias);
   
   spi8_client = spi;
   
   mutex_init(&spi8_lock);
   
   for (i=0; i<DUMMY_BUF_SIZE; i++)
      dummy_buf[i] = 0;
   /* init SPI8 GPIOs */
   rc = gpio_request(FPGA_RESET_GPIO, "fpga_reset");
   if (rc) 
   {
      pr_err("Request FPGA_RESET_GPIO failed, rc=%d\n", rc);
      return -ENODEV;
   }
   rc = gpio_direction_output(FPGA_RESET_GPIO, 0);
   if (rc) 
   {
      pr_err("Unable to set FPGA_RESET_GPIO direction\n");
      goto free_gpio;
   }
   rc = gpio_request(FPGA_CDONE_GPIO, "fpga_cdone");
   if (rc) 
   {
      pr_err("Request FPGA_CDONE_GPIO failed, rc=%d\n", rc);
      goto free_gpio;
   }
   rc = gpio_direction_input(FPGA_CDONE_GPIO);
   if (rc) 
   {
      pr_err("Unable to set FPGA_CDONE_GPIOdirection\n");
      goto free_gpio;
   }
   rc = gpio_request(SPI8_SS, "spi8_ss");
   if (rc) 
   {
      pr_err("Request SPI8_SS failed, rc=%d\n", rc);
      goto free_gpio;
   }
   rc = gpio_direction_output(SPI8_SS, 0);
   if (rc) 
   {
      pr_err("Unable to set SPI8_SS direction\n");
      goto free_gpio;
   }
   /* init FPGA GPIOs */
   for (i=0; i<FPGA_BANK1_CPU_PIO_COUNT; i++)
   {
      rc = gpio_request(fpga_bank1_cpu_pio[i].gpio, fpga_bank1_cpu_pio[i].name);
      if (rc) 
      {
         pr_err("Request FPGA %s failed, rc=%d\n", fpga_bank1_cpu_pio[i].name, rc);
         goto free_gpio;
      }
   }
   for (i=0; i<FPGA_BANK2_CPU_PIO_COUNT; i++)
   {
      rc = gpio_request(fpga_bank2_cpu_pio[i].gpio, fpga_bank2_cpu_pio[i].name);
      if (rc) 
      {
         pr_err("Request FPGA %s failed, rc=%d\n", fpga_bank2_cpu_pio[i].name, rc);
         goto free_gpio;
      }
   }
   for (i=0; i<FPGA_BANK1_PM_PIO_COUNT; i++)
   {  
      fpga_bank1_pm_pio[i].pm_gpio = PM8921_GPIO_PM_TO_SYS(fpga_bank1_pm_pio[i].gpio);
      rc = gpio_request(fpga_bank1_pm_pio[i].pm_gpio, fpga_bank1_pm_pio[i].name);
      if (rc) 
      {
         pr_err("Request FPGA %s failed, rc=%d\n", fpga_bank2_cpu_pio[i].name, rc);
         goto free_gpio;
      }
   }
   
    /* init FPGA clock */
   fpga_clk = clk_get(NULL, "gp2_clk");
   if (IS_ERR(fpga_clk)) 
   {
      pr_err("%s: clk_get failed\n", __func__);
      goto free_gpio;
   }
   clk_set_rate(fpga_clk, 27000000);
   rc = clk_prepare_enable(fpga_clk);
   if (rc) 
   {
      pr_err("unable to enable fpga_clk\n");
      goto free_gpio;
   }
   rc = gpio_request(GPIO_CLOCK, "fpga_clock");
   if (rc) 
   {
      pr_err("Request FPGA_CLOCK failed, rc=%d\n", rc);
      goto free_gpio;
   }

   return 0;

free_gpio:
   gpio_free(FPGA_RESET_GPIO);
   gpio_free(FPGA_CDONE_GPIO);
   gpio_free(SPI8_SS);
   
   for (i=0; i<FPGA_BANK1_CPU_PIO_COUNT; i++)
   {
      gpio_free(fpga_bank1_cpu_pio[i].gpio);
   }
   for (i=0; i<FPGA_BANK2_CPU_PIO_COUNT; i++)
   {
      gpio_free(fpga_bank2_cpu_pio[i].gpio);
   }
   for (i=0; i<FPGA_BANK1_PM_PIO_COUNT; i++)
   {
      gpio_free(fpga_bank1_pm_pio[i].pm_gpio);
   }
   gpio_free(GPIO_CLOCK);
   return -ENODEV;
}

static int __devexit spi8_remove(struct spi_device *spi)
{
   int i;
   
   spi8_client = NULL;
   
   gpio_free(FPGA_RESET_GPIO);
   gpio_free(FPGA_CDONE_GPIO);
   gpio_free(SPI8_SS);
   
   for (i=0; i<FPGA_BANK1_CPU_PIO_COUNT; i++)
   {
      gpio_free(fpga_bank1_cpu_pio[i].gpio);
   }
   for (i=0; i<FPGA_BANK2_CPU_PIO_COUNT; i++)
   {
      gpio_free(fpga_bank2_cpu_pio[i].gpio);
   }
   for (i=0; i<FPGA_BANK1_PM_PIO_COUNT; i++)
   {
      gpio_free(fpga_bank1_pm_pio[i].pm_gpio);
   }
   clk_disable_unprepare(fpga_clk);
   clk_put(fpga_clk);
   gpio_free(GPIO_CLOCK);
   return 0;
}

static int __devinit spi10_probe(struct spi_device *spi)
{
   int rc;
   
   pr_info("%s: %s\n", __func__, spi->modalias);
   
   spi10_client = spi;
   
   mutex_init(&spi10_lock);
   
   rc = gpio_request(SPI10_CS, "spi10_cs");
   if (rc) 
   {
      pr_err("Request SPI10_CS failed, rc=%d\n", rc);
      return -ENODEV;
   }
   rc = gpio_direction_output(SPI10_CS, 0);
   if (rc) 
   {
      pr_err("Unable to set SPI10_CS direction\n");
      goto free_gpio;
   }
   return 0;
free_gpio:
   gpio_free(SPI10_CS);
   return -ENODEV;
}

static int __devexit spi10_remove(struct spi_device *spi)
{
   spi10_client = NULL;
   
   gpio_free(SPI10_CS);
   return 0;
}

static const struct file_operations spi_fpga_fops = {
   .owner         = THIS_MODULE,
   .write         = fpga_image_download,
   .unlocked_ioctl= fpga_ioctl,
};

struct miscdevice spi_fpga_misc = {
   .minor   = MISC_DYNAMIC_MINOR,
   .name    = "spi_fpga",
   .fops    = &spi_fpga_fops,
};

static struct spi_driver spi1_driver = {
   .probe   = spi1_probe,
   .remove  = __devexit_p(spi1_remove),
   .driver  = {
      .name = SPI1_DRIVER_NAME,
      .owner = THIS_MODULE,
      },
};

static struct spi_driver spi8_driver = {
   .probe   = spi8_probe,
   .remove  = __devexit_p(spi8_remove),
   .driver  = {
      .name = SPI8_DRIVER_NAME,
      .owner = THIS_MODULE,
      },
};

static struct spi_driver spi10_driver = {
   .probe   = spi10_probe,
   .remove  = __devexit_p(spi10_remove),
   .driver  = {
      .name = SPI10_DRIVER_NAME,
      .owner = THIS_MODULE,
      },
};

static void __exit msm8960_spi_fpga_exit(void)
{   
   spi_unregister_driver(&spi1_driver);
   spi_unregister_driver(&spi8_driver);
   spi_unregister_driver(&spi10_driver);
   
   misc_deregister(&spi_fpga_misc);
}

static int __init msm8960_spi_fpga_init(void)
{
   int ret;
   
   ret = spi_register_driver(&spi1_driver);
   if (ret) 
   {
      pr_err("%s: spi1 register failed: rc=%d\n", __func__, ret);
   }
   
   ret = spi_register_driver(&spi8_driver);
   if (ret) 
   {
      pr_err("%s: spi8 register failed: rc=%d\n", __func__, ret);
   }
   
   ret = spi_register_driver(&spi10_driver);
   if (ret) 
   {
      pr_err("%s: spi10 register failed: rc=%d\n", __func__, ret);
   }

   misc_register(&spi_fpga_misc);
   
   return ret;
}

module_init(msm8960_spi_fpga_init);
module_exit(msm8960_spi_fpga_exit);

MODULE_DESCRIPTION("FPGA IMAGE DOWNLOAD Driver");
MODULE_LICENSE("GPL");


