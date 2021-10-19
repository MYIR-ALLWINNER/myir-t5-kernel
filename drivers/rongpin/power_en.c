#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
//#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/proc_fs.h>

#include <linux/pinctrl/consumer.h>
#include <linux/sunxi-gpio.h>

#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/string.h>

struct gpio_gpio {
	int gpio_num;
	struct gpio_config gpio_cfg;
};

struct power_en_gpio {
	struct gpio_gpio gpio0;
	struct gpio_gpio gpio1;
	struct gpio_gpio gpio2;
	struct gpio_gpio gpio3;
	struct gpio_gpio gpio4;
	struct gpio_gpio gpio5;
	struct gpio_gpio gpio6;
	struct gpio_gpio gpio7;
	struct gpio_gpio gpio8;
	struct gpio_gpio gpio9;
	int sleep_flag;
//	struct wake_lock rp_wake_lock;
};

static int otg_mode_gpio = 0;
static int otg_pwr_gpio = 0;


static int otg_mode_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t otg_mode_write(struct file *file, const char *buffer,size_t count, loff_t *data)
{
	
	char buf[2]={0};
	char s1[]="0";
	
	if(copy_from_user(&buf[0],buffer,1)){
        	printk("failed to copy data to kernel space\n");
        	return -EFAULT;     
    	}

	if(!strcmp(buf,s1))
		{
			gpio_direction_output(otg_mode_gpio, 0);
			gpio_direction_output(otg_pwr_gpio, 0);
			gpio_set_value(otg_mode_gpio, 0);
			gpio_set_value(otg_pwr_gpio, 0);
			printk("\ndevice mode \n\n");
		}
	else
		{
			gpio_direction_output(otg_mode_gpio, 1);
			gpio_direction_output(otg_pwr_gpio, 1);
			gpio_set_value(otg_mode_gpio, 1);
			gpio_set_value(otg_pwr_gpio, 1);
			printk("\nhost mode \n\n");
		}
		
	return count;
}


static const struct file_operations otg_mode_ops = {
	.owner          = THIS_MODULE,
    .open           = otg_mode_open,
    .write          = otg_mode_write,
    .read           = NULL,
};
 

static int power_en_probe(struct platform_device *pdev)
{
        int ret = 0;
        struct device_node *np = pdev->dev.of_node;
	struct power_en_gpio *data;
	enum of_gpio_flags gpio_flags;
	
	
	static struct proc_dir_entry *root_entry_gpio;
	
	printk("\nrpdzkj %s\n\n",__func__);
	data = devm_kzalloc(&pdev->dev, sizeof(struct power_en_gpio),GFP_KERNEL);
	if (!data) {
                dev_err(&pdev->dev, "failed to allocate memory\n");
                return -ENOMEM;
        }
	memset(data, 0, sizeof(struct power_en_gpio));

        data->gpio0.gpio_num = of_get_named_gpio_flags(np, "gpio0", 0, (enum of_gpio_flags *)(&data->gpio0.gpio_cfg));
        if (!gpio_is_valid(data->gpio0.gpio_num))
                data->gpio0.gpio_num = -1;
        
        data->gpio1.gpio_num = of_get_named_gpio_flags(np, "gpio1", 0, (enum of_gpio_flags *)(&data->gpio1.gpio_cfg));
        if (!gpio_is_valid(data->gpio1.gpio_num))
                data->gpio1.gpio_num = -1;
                
                
        data->gpio2.gpio_num = of_get_named_gpio_flags(np, "gpio2", 0, (enum of_gpio_flags *)(&data->gpio2.gpio_cfg));
        if (!gpio_is_valid(data->gpio2.gpio_num))
                data->gpio2.gpio_num = -1;
                
        data->gpio3.gpio_num = of_get_named_gpio_flags(np, "gpio3", 0, (enum of_gpio_flags *)(&data->gpio3.gpio_cfg));
        if (!gpio_is_valid(data->gpio3.gpio_num))
                data->gpio3.gpio_num = -1;
                
                
        data->gpio4.gpio_num = of_get_named_gpio_flags(np, "gpio4", 0, (enum of_gpio_flags *)(&data->gpio4.gpio_cfg));
        if (!gpio_is_valid(data->gpio4.gpio_num))
                data->gpio4.gpio_num = -1;
        
        data->gpio5.gpio_num = of_get_named_gpio_flags(np, "gpio5", 0, (enum of_gpio_flags *)(&data->gpio5.gpio_cfg));
        if (!gpio_is_valid(data->gpio5.gpio_num))
                data->gpio5.gpio_num = -1;
                
                
        data->gpio6.gpio_num = of_get_named_gpio_flags(np, "gpio6", 0, (enum of_gpio_flags *)(&data->gpio6.gpio_cfg));
        if (!gpio_is_valid(data->gpio6.gpio_num))
                data->gpio6.gpio_num = -1;
                
        data->gpio7.gpio_num = of_get_named_gpio_flags(np, "gpio7", 0, (enum of_gpio_flags *)(&data->gpio7.gpio_cfg));
        if (!gpio_is_valid(data->gpio7.gpio_num))
                data->gpio7.gpio_num = -1;
                
                
        data->gpio8.gpio_num = of_get_named_gpio_flags(np, "gpio8", 0, (enum of_gpio_flags *)(&data->gpio8.gpio_cfg));
        if (!gpio_is_valid(data->gpio8.gpio_num))
                data->gpio8.gpio_num = -1;
       else
          	otg_mode_gpio = data->gpio8.gpio_num;
          
                
        data->gpio9.gpio_num = of_get_named_gpio_flags(np, "gpio9", 0, (enum of_gpio_flags *)(&data->gpio9.gpio_cfg));
        if (!gpio_is_valid(data->gpio9.gpio_num))
                data->gpio9.gpio_num = -1;
       	else
       		otg_pwr_gpio = data->gpio9.gpio_num;
         
                
                
                
        printk("data->gpio0.gpio_cfg.gpio = %d\n",data->gpio0.gpio_cfg.gpio);
        printk("data->gpio0.gpio_cfg.data = %d\n",data->gpio0.gpio_cfg.data);
        printk("data->gpio1.gpio_cfg.gpio = %d\n",data->gpio1.gpio_cfg.gpio);
        printk("data->gpio1.gpio_cfg.data = %d\n",data->gpio1.gpio_cfg.data);
        printk("data->gpio2.gpio_cfg.gpio = %d\n",data->gpio2.gpio_cfg.gpio);
        printk("data->gpio2.gpio_cfg.data = %d\n",data->gpio2.gpio_cfg.data);
        
        printk("data->gpio8.gpio_cfg.gpio = %d\n",data->gpio8.gpio_cfg.gpio);
        printk("data->gpio8.gpio_cfg.data = %d\n",data->gpio8.gpio_cfg.data);
        printk("data->gpio9.gpio_cfg.gpio = %d\n",data->gpio9.gpio_cfg.gpio);
        printk("data->gpio9.gpio_cfg.data = %d\n",data->gpio9.gpio_cfg.data);


		of_property_read_u32(np, "rp_not_deep_leep", &data->sleep_flag);

//	platform_set_drvdata(pdev, data);

	if(data->gpio0.gpio_num != -1){
		ret = gpio_request(data->gpio0.gpio_num, "gpio0");
        	if (ret < 0){
			printk("data->gpio0 request error\n");
		}else{
			gpio_direction_output(data->gpio0.gpio_cfg.gpio, data->gpio0.gpio_cfg.data);
      gpio_set_value(data->gpio0.gpio_cfg.gpio, data->gpio0.gpio_cfg.data);
		}
	}
	
	
	if(data->gpio1.gpio_num != -1){
		ret = gpio_request(data->gpio1.gpio_num, "gpio1");
        	if (ret < 0){
			printk("data->gpio1 request error\n");
		}else{
			gpio_direction_output(data->gpio1.gpio_cfg.gpio, data->gpio1.gpio_cfg.data);
      gpio_set_value(data->gpio1.gpio_cfg.gpio, data->gpio1.gpio_cfg.data);
		}
	}
	
	if(data->gpio2.gpio_num != -1){
		ret = gpio_request(data->gpio2.gpio_num, "gpio2");
        	if (ret < 0){
			printk("data->gpio2 request error\n");
		}else{
			gpio_direction_output(data->gpio2.gpio_cfg.gpio, data->gpio2.gpio_cfg.data);
      gpio_set_value(data->gpio2.gpio_cfg.gpio, data->gpio2.gpio_cfg.data);
		}
	}
	
	
	if(data->gpio3.gpio_num != -1){
		ret = gpio_request(data->gpio3.gpio_num, "gpio3");
        	if (ret < 0){
			printk("data->gpio3 request error\n");
		}else{
			gpio_direction_output(data->gpio3.gpio_cfg.gpio, data->gpio3.gpio_cfg.data);
      gpio_set_value(data->gpio3.gpio_cfg.gpio, data->gpio3.gpio_cfg.data);
		}
	}
	
	
	
	if(data->gpio4.gpio_num != -1){
		ret = gpio_request(data->gpio4.gpio_num, "gpio4");
        	if (ret < 0){
			printk("data->gpio4 request error\n");
		}else{
			gpio_direction_output(data->gpio4.gpio_cfg.gpio, data->gpio4.gpio_cfg.data);
      gpio_set_value(data->gpio4.gpio_cfg.gpio, data->gpio4.gpio_cfg.data);
		}
	}
	
	
	if(data->gpio5.gpio_num != -1){
		ret = gpio_request(data->gpio5.gpio_num, "gpio5");
        	if (ret < 0){
			printk("data->gpio5 request error\n");
		}else{
			gpio_direction_output(data->gpio5.gpio_cfg.gpio, data->gpio5.gpio_cfg.data);
      gpio_set_value(data->gpio5.gpio_cfg.gpio, data->gpio5.gpio_cfg.data);
		}
	}
	
	if(data->gpio6.gpio_num != -1){
		ret = gpio_request(data->gpio6.gpio_num, "gpio6");
        	if (ret < 0){
			printk("data->gpio6 request error\n");
		}else{
			gpio_direction_output(data->gpio6.gpio_cfg.gpio, data->gpio6.gpio_cfg.data);
      gpio_set_value(data->gpio6.gpio_cfg.gpio, data->gpio6.gpio_cfg.data);
		}
	}
	
	
	if(data->gpio7.gpio_num != -1){
		ret = gpio_request(data->gpio7.gpio_num, "gpio7");
        	if (ret < 0){
			printk("data->gpio7 request error\n");
		}else{
			gpio_direction_output(data->gpio7.gpio_cfg.gpio, data->gpio7.gpio_cfg.data);
      gpio_set_value(data->gpio7.gpio_cfg.gpio, data->gpio7.gpio_cfg.data);
		}
	}
	
	
	if(data->gpio8.gpio_num != -1){
		ret = gpio_request(data->gpio8.gpio_num, "gpio8");
        	if (ret < 0){
			printk("data->gpio8 request error\n");
		}else{
			gpio_direction_output(data->gpio8.gpio_cfg.gpio, data->gpio8.gpio_cfg.data);
      gpio_set_value(data->gpio8.gpio_cfg.gpio, data->gpio8.gpio_cfg.data);
		}
	}
	
	
	if(data->gpio9.gpio_num != -1){
		ret = gpio_request(data->gpio9.gpio_num, "gpio9");
        	if (ret < 0){
			printk("data->gpio9 request error\n");
		}else{
			gpio_direction_output(data->gpio9.gpio_cfg.gpio, data->gpio9.gpio_cfg.data);
      gpio_set_value(data->gpio9.gpio_cfg.gpio, data->gpio9.gpio_cfg.data);
		}
	}


	if(data->sleep_flag != 0){
//		wake_lock_init(&data->rp_wake_lock,WAKE_LOCK_SUSPEND, "rpdzkj_no_deep_sleep");
//		wake_lock(&data->rp_wake_lock);
	}
	
	
    	/* create node */
    	root_entry_gpio = proc_mkdir("otg_mode", NULL);
    	
		proc_create("mode", 0666 , root_entry_gpio , &otg_mode_ops);
	
	printk("\nrpdzkj %s end\n\n",__func__);
	
        return 0;
}

static int power_en_remove(struct platform_device *pdev)
{
        struct power_en_gpio *data = platform_get_drvdata(pdev);


	if(data->gpio0.gpio_num != -1){
		gpio_direction_output(data->gpio0.gpio_num, 0);
		gpio_free(data->gpio0.gpio_num);
	}
	if(data->gpio1.gpio_num != -1){
		gpio_direction_output(data->gpio1.gpio_num, 0);
		gpio_free(data->gpio1.gpio_num);
	}
	if(data->gpio2.gpio_num != -1){
		gpio_direction_output(data->gpio2.gpio_num, 0);
		gpio_free(data->gpio2.gpio_num);
	}
	if(data->gpio3.gpio_num != -1){
		gpio_direction_output(data->gpio3.gpio_num, 0);
		gpio_free(data->gpio3.gpio_num);
	}
	if(data->gpio4.gpio_num != -1){
		gpio_direction_output(data->gpio4.gpio_num, 0);
		gpio_free(data->gpio4.gpio_num);
	}
	if(data->gpio5.gpio_num != -1){
		gpio_direction_output(data->gpio5.gpio_num, 0);
		gpio_free(data->gpio5.gpio_num);
	}
	if(data->gpio6.gpio_num != -1){
		gpio_direction_output(data->gpio6.gpio_num, 0);
		gpio_free(data->gpio6.gpio_num);
	}
	if(data->gpio7.gpio_num != -1){
		gpio_direction_output(data->gpio7.gpio_num, 0);
		gpio_free(data->gpio7.gpio_num);
	}
	if(data->gpio8.gpio_num != -1){
		gpio_direction_output(data->gpio8.gpio_num, 0);
		gpio_free(data->gpio8.gpio_num);
	}
	if(data->gpio9.gpio_num != -1){
		gpio_direction_output(data->gpio9.gpio_num, 0);
		gpio_free(data->gpio9.gpio_num);
	}
	if(data->sleep_flag != 0){
//		wake_unlock(&data->rp_wake_lock);
	}

        return 0;
}

#ifdef CONFIG_PM 
static int power_en_suspend(struct device *dev) 
{ 
	struct platform_device *pdev = to_platform_device(dev);
        struct power_en_gpio *data = platform_get_drvdata(pdev);

#if 0 
	if(data->gpio0.gpio_num != -1){
		gpio_direction_output(data->gpio0.gpio_num, 0);
		gpio_set_value(data->gpio0.gpio_num,0);
	}
	if(data->gpio1.gpio_num != -1){
		gpio_direction_output(data->gpio1.gpio_num, 0);
		gpio_set_value(data->gpio1.gpio_num,0);
	}
	if(data->gpio2.gpio_num != -1){
		gpio_direction_output(data->gpio2.gpio_num, 0);
		gpio_set_value(data->gpio2.gpio_num,0);
	}
	if(data->gpio3.gpio_num != -1){
		gpio_direction_output(data->gpio3.gpio_num, 0);
		gpio_set_value(data->gpio3.gpio_num,0);
	}
	if(data->gpio4.gpio_num != -1){
		gpio_direction_output(data->gpio4.gpio_num, 0);
		gpio_set_value(data->gpio4.gpio_num,0);
	}
	if(data->gpio5.gpio_num != -1){
		gpio_direction_output(data->gpio5.gpio_num, 0);
		gpio_set_value(data->gpio5.gpio_num,0);
	}
	if(data->gpio6.gpio_num != -1){
		gpio_direction_output(data->gpio6.gpio_num, 0);
		gpio_set_value(data->gpio6.gpio_num,0);
	}
	if(data->gpio7.gpio_num != -1){
		gpio_direction_output(data->gpio7.gpio_num, 0);
		gpio_set_value(data->gpio7.gpio_num,0);
	}
	if(data->gpio8.gpio_num != -1){
		gpio_direction_output(data->gpio8.gpio_num, 0);
		gpio_set_value(data->gpio8.gpio_num,0);
	}
	if(data->gpio9.gpio_num != -1){
		gpio_direction_output(data->gpio9.gpio_num, 0);
		gpio_set_value(data->gpio9.gpio_num,0);
	}
#endif
        return 0; 
} 
 
static int power_en_resume(struct device *dev) 
{ 
	struct platform_device *pdev = to_platform_device(dev);
        struct power_en_gpio *data = platform_get_drvdata(pdev);
 
#if 0 
	if(data->gpio0.gpio_num != -1){
			gpio_direction_output(data->gpio0.gpio_num, data->gpio0.gpio_cfg.data);
        		gpio_set_value(data->gpio0.gpio_num, data->gpio0.gpio_cfg.data);
	}
      
	if(data->gpio1.gpio_num != -1){
			gpio_direction_output(data->gpio1.gpio_num, data->gpio1.gpio_cfg.data);
        	gpio_set_value(data->gpio1.gpio_num, data->gpio1.gpio_cfg.data);
	}
	
	if(data->gpio2.gpio_num != -1){
			//gpio_direction_output(data->gpio2.gpio_num, 0);
        		//gpio_set_value(data->gpio2.gpio_num, 0);
			//msleep(100);
			gpio_direction_output(data->gpio2.gpio_num, data->gpio2.gpio_cfg.data);
        	gpio_set_value(data->gpio2.gpio_num, data->gpio2.gpio_cfg.data);
	}
        
	if(data->gpio3.gpio_num != -1){
			gpio_direction_output(data->gpio3.gpio_num, data->gpio3.gpio_cfg.data);
  			gpio_set_value(data->gpio3.gpio_num, data->gpio3.gpio_cfg.data);
	}
  
	if(data->gpio4.gpio_num != -1){
			gpio_direction_output(data->gpio4.gpio_num, data->gpio4.gpio_cfg.data);
        		gpio_set_value(data->gpio4.gpio_num, data->gpio4.gpio_cfg.data);
	}
	if(data->gpio5.gpio_num != -1){
			gpio_direction_output(data->gpio5.gpio_num, data->gpio5.gpio_cfg.data);
  			gpio_set_value(data->gpio5.gpio_num, data->gpio5.gpio_cfg.data);
	}
	
	if(data->gpio6.gpio_num != -1){
			gpio_direction_output(data->gpio6.gpio_num, data->gpio6.gpio_cfg.data);
        	gpio_set_value(data->gpio6.gpio_num, data->gpio6.gpio_cfg.data);
	}
	if(data->gpio7.gpio_num != -1){
			gpio_direction_output(data->gpio7.gpio_num, data->gpio7.gpio_cfg.data);
        		gpio_set_value(data->gpio7.gpio_num, data->gpio7.gpio_cfg.data);
	}
	if(data->gpio8.gpio_num != -1){
			gpio_direction_output(data->gpio8.gpio_num, data->gpio8.gpio_cfg.data);
  			gpio_set_value(data->gpio8.gpio_num, data->gpio8.gpio_cfg.data);
	}
	
	if(data->gpio9.gpio_num != -1){
			gpio_direction_output(data->gpio9.gpio_num, data->gpio9.gpio_cfg.data);
        	gpio_set_value(data->gpio9.gpio_num, data->gpio9.gpio_cfg.data);
	}
#endif
        return 0; 
} 


static const struct dev_pm_ops power_en_pm_ops = { 
        .suspend        = power_en_suspend, 
        .resume         = power_en_resume, 
}; 
#endif

static const struct of_device_id power_en_of_match[] = {
        { .compatible = "pw-en-gpio" },
        { }
};

static struct platform_driver power_en_driver = {
        .probe = power_en_probe,
        .remove = power_en_remove,
        .driver = {
                .name           = "pw-en-gpio",
                .of_match_table = of_match_ptr(power_en_of_match),
#ifdef CONFIG_PM
                .pm     = &power_en_pm_ops,
#endif

        },
};

module_platform_driver(power_en_driver);

MODULE_LICENSE("GPL");
