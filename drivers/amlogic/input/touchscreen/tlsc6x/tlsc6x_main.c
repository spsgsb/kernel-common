/*
 * This software is licensed under the terms of the GNU General Public 
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * * VERSION      	DATE			AUTHOR          Note
 * 
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>

#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/irq.h>

#include <linux/hrtimer.h>

#include <linux/sysfs.h>

#if defined(CONFIG_ADF)
#include <linux/notifier.h>
#include <video/adf_notifier.h>
#elif defined(XCONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/miscdevice.h>
#include <linux/pm_wakeup.h>

#include "tlsc6x_main.h"

#include <linux/proc_fs.h>

DEFINE_MUTEX(i2c_rw_access);

#undef TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	1
#define	TS_MAX_FINGER		2

struct tlsc6x_platform_data{
	u32 irq_gpio_number;
	u32 reset_gpio_number;
	u32 use_polling;
	const char *vdd_name;
	u32 virtualkeys[12];
	u32 TP_MAX_X;
	u32 TP_MAX_Y;
};

static struct workqueue_struct *tlsc6x_wq;

#undef TP_PROXIMITY_SENSOR
#ifdef TP_PROXIMITY_SENSOR
#define LTR_IOCTL_MAGIC 0x1C
#define LTR_IOCTL_GET_PFLAG _IOR(LTR_IOCTL_MAGIC, 1, int)
#define LTR_IOCTL_GET_LFLAG _IOR(LTR_IOCTL_MAGIC, 2, int)
#define LTR_IOCTL_SET_PFLAG _IOW(LTR_IOCTL_MAGIC, 3, int)
#define LTR_IOCTL_SET_LFLAG _IOW(LTR_IOCTL_MAGIC, 4, int)
#define LTR_IOCTL_GET_DATA _IOW(LTR_IOCTL_MAGIC, 5, unsigned char)

static int PROXIMITY_SWITCH=0;
static int PROXIMITY_STATE=0;

static struct spinlock proximity_switch_lock;
static struct spinlock proximity_state_lock;
#endif

#define TS_NAME	   	       	"tlsc6x_ts"
#define MAX_CHIP_ID         (10)
#define MAX_VENDOR_ID       (45)

unsigned char tlsc6x_chip_name[MAX_CHIP_ID][20] = {
	"null", "tlsc6206a", "0x6306", "tlsc6206", "tlsc6324",
	"tlsc6332", "tlsc6440", "tlsc6432", "tlsc6424", "tlsc6448",
};

unsigned char tlsc6x_vendor_name[MAX_VENDOR_ID][20] = {
	"null",          "xufang",     "xuri",     "yuye",      "tianyi",
	"minglang",      "duoxinda",   "zhenhua",  "jitegao",   "guangjishengtai",
	"shengguang",    "xintiantong","xinyou",   "yanqi",     "zhongcheng",
	"xinmaoxin",     "zhenzhixin", "helitai",  "huaxin",    "lihaojie",
	"jiandong",      "xinpengda",  "jiake",    "yijian",    "yixing",
	"zhongguangdian","hongzhan",   "huaxingda","dongjianhuanyu","dawosi",
	"dacheng",       "mingwangda", "huangze",  "jinxinxiang","gaoge",
	"zhihui",        "miaochu",    "qicai",    "zhenghai",  "hongfazhan",
	"lianchuang",    "saihua",     "keleli",   "weiyi",     "futuo",
};

int g_is_telink_comp = 0;

static struct tlsc6x_data *g_tp_drvdata = NULL;
static struct i2c_client *this_client;

struct wakeup_source tlsc6x_wakelock;

#if defined(CONFIG_ADF)
static int tlsc6x_suspend(void);
static int tlsc6x_resume(void);
#elif defined(XCONFIG_FB)
static int tlsc6x_fb_suspend(void);
static int tlsc6x_fb_resume(void);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void tlsc6x_ts_suspend(struct early_suspend *handler);
static void tlsc6x_ts_resume(struct early_suspend *handler);
#endif

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct tlsc6x_data {
    struct input_dev *input_dev;
    struct input_dev *ps_input_dev;
    struct i2c_client	*client;
    struct hrtimer timer;
    struct work_struct work;
    spinlock_t irq_lock;
    struct ts_event	event;
#if defined(CONFIG_ADF) || defined(XCONFIG_FB)
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend	early_suspend;
#endif
	struct work_struct       resume_work;
    struct workqueue_struct *tp_resume_workqueue;
    int irq_gpio_number;
    int reset_gpio_number;
    int isVddAlone;
    bool irq_disabled;
    bool suspended;
    struct regulator *reg_vdd;
    struct tlsc6x_platform_data	*platform_data;
};

#ifdef TOUCH_VIRTUAL_KEYS

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct tlsc6x_data *data = i2c_get_clientdata(this_client);
	struct tlsc6x_platform_data *pdata = data->platform_data;
	return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
		,__stringify(EV_KEY), __stringify(KEY_APPSELECT),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
		,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
		,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.tlsc6x_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

#endif

int tlsc6x_i2c_read_nolock(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	if (client == NULL) {
		TLSC_ERROR("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (readlen > 0) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
				 },
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				TLSC_ERROR("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
				TLSC_ERROR("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
				       writelen);
			}else {
				ret = i2c_transfer(client->adapter, &msgs[1], 1);
				if (ret < 0) {
					TLSC_ERROR("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
					TLSC_ERROR("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
					       writelen);
				}
			}
		} else {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				TLSC_ERROR("[IIC]: i2c_transfer(read) error, ret=%d, rlen=%d, wlen=%d!!", ret, readlen,
				       writelen);
			}
		}
	}

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	int offset;
	u8 regBuf[2];
	int subLen;
	int retry;
	

	/* lock in this function so we can do direct mode iic transfer in debug fun */
	mutex_lock(&i2c_rw_access);
	offset = 0;
	while(readlen > 0){
	        if(2 == writelen) {
	        regBuf[0] = (u8)(writebuf[0]);
                regBuf[1] = (u8)(writebuf[1]+offset);
                }
                else if(1 == writelen)  {
                        regBuf[0] = (u8)(writebuf[0]+offset);
                }

	        if(readlen > MAX_TRX_LEN){
	            readlen -= MAX_TRX_LEN;
	            subLen = MAX_TRX_LEN;
	        }else{
	            subLen = readlen;
	            readlen = 0;
	        }

		retry = 0;
		while (tlsc6x_i2c_read_nolock(client, regBuf, writelen, &readbuf[offset],subLen) < 0) {
			if (retry++ == 3) {
				ret = -1;
				break;
			}
		}
		offset += MAX_TRX_LEN;
		if (ret < 0) {
			break;
		}
	}

	mutex_unlock(&i2c_rw_access);

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_write_nolock(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	if (client == NULL) {
		TLSC_ERROR("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0) {
			TLSC_ERROR("[IIC]: i2c_transfer(write) error, ret=%d!!\n", ret);
		}
	}

	return ret;

}

/* fail : <0 */
int tlsc6x_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int retry = 0;
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	for (retry = 0; retry < 3; retry ++) {
	ret = tlsc6x_i2c_write_nolock(client, writebuf, writelen);
		if (ret >= 0) {
			break;
		}
	}
	mutex_unlock(&i2c_rw_access);

	return ret;

}

// fail : <0
int tlsc6x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return tlsc6x_i2c_write(client, buf, sizeof(buf));
}

// fail : <0
int tlsc6x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return tlsc6x_i2c_read(client, &regaddr, 1, regvalue, 1);
}

#ifdef TOUCH_VIRTUAL_KEYS
static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};
#endif

void tlsc_irq_disable(struct tlsc6x_data *drvdata)
{
	unsigned long irqflags;

	spin_lock_irqsave(&drvdata->irq_lock, irqflags);
	if (!drvdata->irq_disabled) {
		disable_irq_nosync(drvdata->client->irq);
		drvdata->irq_disabled = true;
	}
	spin_unlock_irqrestore(&drvdata->irq_lock, irqflags);
}


void tlsc_irq_enable(struct tlsc6x_data *drvdata)
{
	unsigned long irqflags;

	spin_lock_irqsave(&drvdata->irq_lock, irqflags);
	if (drvdata->irq_disabled) {
		enable_irq(drvdata->client->irq);
		drvdata->irq_disabled = false;
	}
	spin_unlock_irqrestore(&drvdata->irq_lock, irqflags);
}

#ifdef TOUCH_VIRTUAL_KEYS

static void tlsc6x_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    TLSC_INFO("%s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if(properties_kobj){
        ret = sysfs_create_group(properties_kobj, &properties_attr_group);
    }
    if(!properties_kobj || ret){
        TLSC_ERROR("failed to create board_properties\n");
    }
}

#endif


static void tlsc6x_clear_report_data(struct tlsc6x_data *drvdata)
{
    #if MULTI_PROTOCOL_TYPE_B
    int i;

    for(i = 0; i<TS_MAX_FINGER; i++){
        input_mt_slot(drvdata->input_dev, i);
        input_mt_report_slot_state(drvdata->input_dev, MT_TOOL_FINGER, false);
        input_report_abs(drvdata->input_dev, ABS_MT_PRESSURE, 0);
    }
    input_mt_sync_frame(drvdata->input_dev);
    #endif

    input_report_key(drvdata->input_dev, BTN_TOUCH, 0);

    #if !MULTI_PROTOCOL_TYPE_B
    input_report_abs(drvdata->input_dev, ABS_MT_PRESSURE, 0);
    input_mt_sync(drvdata->input_dev);
    #endif
    input_sync(drvdata->input_dev);
}

static int tlsc6x_update_data(struct tlsc6x_data *data)
{
    //struct tlsc6x_data *data = i2c_get_clientdata(this_client);
    struct ts_event *event = &data->event;
    u8 buf[20] = {0};
    int ret = -1;
    int i;
    u16 x , y;
    u8 ft_pressure , ft_size;

    ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 18);
    if(ret < 0){
        TLSC_ERROR("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
        return ret;
    }

    memset(event, 0, sizeof(struct ts_event));
    event->touch_point = buf[2] & 0x07;
#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        spin_lock(&proximity_state_lock);

        if (0xC0 == buf[1]){
            PROXIMITY_STATE = 1; // 1;  near
        }else if (0xE0 == buf[1]){
            PROXIMITY_STATE = 0; // 0;  far-away
        }/*else{
            tlsc6x_write_reg(this_client,0xb0, 0x01);
        }*/
        spin_unlock(&proximity_state_lock);
        input_report_abs(data->ps_input_dev, ABS_DISTANCE, PROXIMITY_STATE?0:1);
        input_report_key(data->ps_input_dev, BTN_TOUCH, 0);
    #if !MULTI_PROTOCOL_TYPE_B
        input_mt_sync(data->ps_input_dev);
    #endif
        input_sync(data->ps_input_dev);
    }
#endif
    for(i = 0; i < TS_MAX_FINGER; i++){
        if((buf[6*i+3] & 0xc0) == 0xc0){
            continue;
        }
        x = (s16)(buf[6*i+3] & 0x0F)<<8 | (s16)buf[6*i+4];	
        y = (s16)(buf[6*i+5] & 0x0F)<<8 | (s16)buf[6*i+6];
        ft_pressure = buf[6*i+7];
        if(ft_pressure > 127){
            ft_pressure = 127;
        }
        ft_size = (buf[6*i+8]>>4) & 0x0F;
        if((buf[6*i+3] & 0x40) == 0x0){
            #if MULTI_PROTOCOL_TYPE_B
                input_mt_slot(data->input_dev, buf[6*i+5]>>4);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
            #else
                input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, buf[6 * i + 5] >> 4);
            #endif
                input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
                input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
                input_report_abs(data->input_dev, ABS_MT_PRESSURE, ft_pressure);
                input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
                input_report_key(data->input_dev, BTN_TOUCH, 1);
		    #if !MULTI_PROTOCOL_TYPE_B
                input_mt_sync(data->input_dev);
		    #endif
        }else{
		#if MULTI_PROTOCOL_TYPE_B
                input_mt_slot(data->input_dev, buf[6*i+5]>>4);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		#endif
        }
    }
    if(0 == event->touch_point) {		
        tlsc6x_clear_report_data(data);
    }
#if MULTI_PROTOCOL_TYPE_B
    input_mt_sync_frame(data->input_dev);
#endif
    input_sync(data->input_dev);

    return 0;

}

#ifdef TLSC_ESD_HELPER_EN
static int tpd_esd_flag = 0;
static struct hrtimer tpd_esd_kthread_timer;
static DECLARE_WAIT_QUEUE_HEAD(tpd_esd_waiter);
#endif
#if 0
static int touch_event_handler(void *unused)
{
    //struct sched_param param = { .sched_priority = 5 };
    //sched_setscheduler(current, SCHED_RR, &param);
    DEFINE_WAIT_FUNC(wait, woken_wake_function);

    add_wait_queue(&waiter, &wait);
    do{
        //set_current_state(TASK_INTERRUPTIBLE);
        //wait_event_interruptible(waiter, (0 != tpd_flag));
        if (0 == tpd_flag) {
             wait_woken(&wait, TASK_INTERRUPTIBLE,
                               MAX_SCHEDULE_TIMEOUT);
             continue;
        }
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);

        tlsc6x_update_data();

    } while (!kthread_should_stop());

    remove_wait_queue(&waiter, &wait);
    return 0;
}
#endif
static void tlsc6x_work_func(struct work_struct *work)
{
	struct tlsc6x_data *data;

	data = container_of(work, struct tlsc6x_data, work);

	tlsc6x_update_data(data);

	if (data->platform_data->use_polling == 0)
		tlsc_irq_enable(data);
}

static enum hrtimer_restart tlsc6x_timer_handler(struct hrtimer *timer)
{
    struct tlsc6x_data *data;

    data = container_of(timer, struct tlsc6x_data, timer);

    queue_work(tlsc6x_wq, &data->work);
    hrtimer_start(&data->timer,
    				ktime_set(0, (TLSC_POLL_TIME+6)*1000000),
					HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

static irqreturn_t tlsc6x_interrupt(int irq, void *dev_id)
{
    struct tlsc6x_data *data = dev_id;

    //TLSC_INFO("== %s irq:%d ==\n", __FUNCTION__, irq);

    tlsc_irq_disable(data);

    queue_work(tlsc6x_wq, &data->work);
    return IRQ_HANDLED;
}

static void tlsc6x_tpd_reset(void)
{
    struct tlsc6x_platform_data *pdata = g_tp_drvdata->platform_data;

    gpio_direction_output(pdata->reset_gpio_number, 1);
    msleep(1);
    gpio_set_value(pdata->reset_gpio_number, 0);
    msleep(20);
    gpio_set_value(pdata->reset_gpio_number, 1);
    msleep(30);
}

static unsigned char suspend_flags = 0;

#if defined(CONFIG_ADF)
static int tlsc6x_suspend(void)
{
	int ret = -1;

    TLSC_INFO("==%s==\n", __FUNCTION__);

#ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        suspend_flags = 0;
        return 0;
    }
#endif

    tlsc_irq_disable(g_tp_drvdata);
    ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
    if(ret<0){
    	TLSC_ERROR("setup suspend fail!\n");
    }
    suspend_flags =1;
    tlsc6x_clear_report_data(g_tp_drvdata);
    return 0;
}

static int tlsc6x_resume(void)
{
	#ifdef TLSC_ESD_HELPER_EN
    hrtimer_start(&tpd_esd_kthread_timer,  ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH && (suspend_flags == 0)){
        return 0;
    }
#endif

    queue_work(g_tp_drvdata->tp_resume_workqueue, &g_tp_drvdata->resume_work);
    return 0;
}
#elif defined(XCONFIG_FB)
static int tlsc6x_fb_suspend(void)
{
   int ret = -1;

    TLSC_INFO("==%s==\n", __FUNCTION__);

	if (g_tp_drvdata->suspended) {
		TLSC_INFO("Already in suspend state.\n");
		return 0;
	}
#ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        suspend_flags = 0;
        enable_irq_wake(this_client->irq);
        g_tp_drvdata->suspended = true;
        return 0;
    }
#endif

    tlsc_irq_disable(g_tp_drvdata);
    ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
    if(ret < 0){
    	TLSC_ERROR("setup suspend fail!\n");
    }
    suspend_flags = 1;
    tlsc6x_clear_report_data(g_tp_drvdata);
    g_tp_drvdata->suspended = true;

    return 0;
}

static int tlsc6x_fb_resume(void)
{
#ifdef TLSC_ESD_HELPER_EN
    hrtimer_start(&tpd_esd_kthread_timer,  ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH && (suspend_flags == 0)){
        return 0;
    }
#endif

    queue_work(g_tp_drvdata->tp_resume_workqueue, &g_tp_drvdata->resume_work);

    return 0;
}

#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void tlsc6x_ts_suspend(struct early_suspend *handler)
{
   int ret = -1;

    TLSC_INFO("==%s==\n", __FUNCTION__);

#ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        suspend_flags = 0;
        return;
    }
#endif

    tlsc_irq_disable(g_tp_drvdata);
    ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
    if(ret<0){
    	TLSC_ERROR("setup suspend fail!\n");
    }
    suspend_flags =1;
    tlsc6x_clear_report_data(g_tp_drvdata);

return;
}

static void tlsc6x_ts_resume(struct early_suspend *handler)
{
#ifdef TLSC_ESD_HELPER_EN
    hrtimer_start(&tpd_esd_kthread_timer,  ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH && (suspend_flags == 0)){
        return;
    }
#endif

    queue_work(g_tp_drvdata->tp_resume_workqueue, &g_tp_drvdata->resume_work);

}

#endif

static void tlsc6x_resume_work(struct work_struct *work)
{
    TLSC_INFO("==%s==\n", __FUNCTION__);

	if (g_tp_drvdata->suspended == false) {
		TLSC_INFO("Already in awake state");
		return;
	}

    tlsc6x_tpd_reset();
    tlsc6x_clear_report_data(g_tp_drvdata);

#ifdef TP_PROXIMITY_SENSOR
    if(PROXIMITY_SWITCH){
        msleep(100);//wait for stable
        tlsc6x_write_reg(this_client,0xb0, 0x01);
        if (suspend_flags == 0) {
        	disable_irq_wake(this_client->irq);
        	g_tp_drvdata->suspended = false;
        }
    }
#endif
	
    tlsc_irq_enable(g_tp_drvdata);

    suspend_flags = 0;
    g_tp_drvdata->suspended = false;
}


#if defined(CONFIG_ADF)
/*
 * touchscreen's suspend and resume state should rely on screen state,
 * as fb_notifier and early_suspend are all disabled on our platform,
 * we can only use adf_event now
 */
static int ts_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{

	struct adf_notifier_event *event = data;
	int adf_event_data;

	if (action != ADF_EVENT_BLANK)
		return NOTIFY_DONE;

	adf_event_data = *(int *)event->data;
	TLSC_INFO("receive adf event with adf_event_data=%d", adf_event_data);

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		tlsc6x_resume();
		break;
	case DRM_MODE_DPMS_OFF:
		tlsc6x_suspend();
		break;
	default:
		TLSC_INFO("receive adf event with error data, adf_event_data=%d",
			adf_event_data);
		break;
	}

	return NOTIFY_OK;
}

#elif defined(XCONFIG_FB)
/*****************************************************************************
*  Name: fb_notifier_callback
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct tlsc6x_data *tlsc6x_data = container_of(self, struct tlsc6x_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && tlsc6x_data && tlsc6x_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			TLSC_INFO("resume notifier.\n");
			tlsc6x_fb_resume();
		} else if (*blank == FB_BLANK_POWERDOWN) {
			TLSC_INFO("suspend notifier.\n");
			tlsc6x_fb_suspend();
		}
	}

	return 0;
}

#endif

static int tlsc6x_hw_init(struct tlsc6x_data *drvdata)
{
    struct tlsc6x_platform_data *pdata = drvdata->platform_data;
    int    ret;

    TLSC_INFO("%s [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
    ret = gpio_request(pdata->irq_gpio_number, NULL);
    if(ret < 0){
        goto OUT;
    }
    ret = gpio_request(pdata->reset_gpio_number, NULL);
    if(ret < 0){
        goto OUT;
    }
    gpio_direction_output(pdata->reset_gpio_number, 1);
    gpio_direction_input(pdata->irq_gpio_number);

    drvdata->reg_vdd = NULL;
    tlsc6x_tpd_reset();

    return 0;

OUT:
    return ret;
}


#ifdef CONFIG_OF
static struct tlsc6x_platform_data *tlsc6x_parse_dt(struct device *dev)
{
    struct tlsc6x_platform_data *pdata;
    struct device_node *np = dev->of_node;
    int ret;

    pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
    if (!pdata) {
        TLSC_ERROR("Could not allocate struct tlsc6x_platform_data");
        return NULL;
    }

    pdata->irq_gpio_number = of_get_named_gpio(np, "irq-gpio", 0);
    pdata->reset_gpio_number = of_get_named_gpio(np, "reset-gpio", 0);

    ret = of_property_read_u32(np, "polling-mode", &pdata->use_polling);
	if (ret) {
		//The default mode is irq.
		pdata->use_polling = 0;
	}

#ifdef TOUCH_VIRTUAL_KEYS
    ret = of_property_read_u32_array(np, "virtualkeys", pdata->virtualkeys,12);
    if(ret){
        TLSC_ERROR("fail to get virtualkeys\n");
        goto fail;
    }
#endif
    ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
    if(ret){
        TLSC_ERROR("fail to get TP_MAX_X\n");
        goto fail;
    }
    ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
    if(ret){
        TLSC_ERROR("fail to get TP_MAX_Y\n");
        goto fail;
    }

    return pdata;
fail:
    kfree(pdata);
    return NULL;
}
#endif

#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)

int auto_upd_busy = 0;
//0:sucess
//1: no file OR open fail
//2: wrong file size OR read error
//-1:op-fial
// TODO
int tlsc6x_proc_cfg_update(u8 *dir, int behave)
{
    int ret = 1;
    u8 buf[256];
    u32 fileSize;
    loff_t pos = 0;
    u8 regval;
    mm_segment_t old_fs;
    static struct file *file = NULL;

    TLSC_INFO("proc-file:%s\n", dir);

    file = filp_open(dir, O_RDONLY, 0);
    if(IS_ERR(file)){
    	TLSC_ERROR("proc-file:open error!\n");
    }else{
        ret = 2;
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        fileSize = file->f_op->llseek(file, 0, SEEK_END);
        TLSC_INFO("proc-file, size:%d\n", fileSize);
        if(204 == fileSize){
            tlsc6x_read_reg(this_client, 0x11,&regval);
            file->f_op->llseek(file, 0, SEEK_SET);
            if(204 == file->f_op->read(file, (char *)buf, 204, &file->f_pos)){
            	TLSC_INFO("proc-file, read ok1!\n");
                tlsc6x_read_reg(this_client, 0x12,&regval);
                ret = 3;
            }else if(204 == vfs_read(file, buf, 204, &pos)){
            	TLSC_INFO("proc-file, read ok2!\n");
                tlsc6x_read_reg(this_client, 0x13,&regval);
                ret = 3;
            }
            if(3 == ret){
                auto_upd_busy = 1;
                tlsc_irq_disable(g_tp_drvdata);
                msleep(1500);
                tlsc6x_read_reg(this_client, 0x14,&regval);
                __pm_wakeup_event(&tlsc6x_wakelock, 2000);
                if(0 == behave){
                    ret = tlsx6x_update_running_cfg((u16*)buf);
                }else{
                    ret = tlsx6x_update_burn_cfg((u16*)buf);
                    tlsc6x_tpd_reset();
                }
                tlsc_irq_enable(g_tp_drvdata);
                auto_upd_busy = 0;
            }else{
            	TLSC_ERROR("proc-file, read error!\n");
            }
        }
        filp_close(file, NULL);
        set_fs(old_fs);
    }

    return ret;
}

#endif


#ifdef TLSC_APK_DEBUG    // MUST:(LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
unsigned char proc_out_len;
unsigned char proc_out_buf[256];
static struct proc_dir_entry *tlsc6x_proc_entry = NULL;
static ssize_t tlsc6x_proc_read(struct file *filp, char __user *page, size_t len, loff_t *pos)
{
    if(0 == proc_out_len){
        return -EFAULT;
    }

    if(copy_to_user(page,proc_out_buf,proc_out_len)){
        return -EFAULT;
    }

    return proc_out_len;
}

static ssize_t tlsc6x_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *ops)
{
    int buflen = len;
    unsigned char local_buf[256];

    if(buflen > 255){
        return -EFAULT;
    }
	
    if(copy_from_user(&local_buf, buff, buflen)){
    	TLSC_ERROR("%s:copy from user error\n", __func__);
        return -EFAULT;
    }

    // format:cmd+para+data0+data1+data2...
    switch(local_buf[0]){
    case 0: // cfg version
        proc_out_buf[0] = g_tlsc6x_cfg_ver;
        proc_out_buf[1] = g_tlsc6x_cfg_ver>>8;
        proc_out_buf[2] = g_tlsc6x_cfg_ver>>16;
        proc_out_buf[3] = g_tlsc6x_cfg_ver>>24;
        proc_out_len = 4;
        break;
    case 1:
        local_buf[buflen] = '\0';
        if(tlsc6x_proc_cfg_update(&local_buf[2], 0)){
            len = -EIO;
        }
        break;
    case 2:
        local_buf[buflen] = '\0';
        if(tlsc6x_proc_cfg_update(&local_buf[2], 1)){
            len = -EIO;
        }
        break;
    default:
        break;
    }

    return len;
}

static struct file_operations tlsc6x_proc_ops = {
    .owner = THIS_MODULE,
    .read = tlsc6x_proc_read,
    .write = tlsc6x_proc_write,
};
int tlsc6x_create_apk_debug_channel(struct i2c_client * client)
{
    tlsc6x_proc_entry = proc_create_data("tlsc6x-debug", S_IRUGO | S_IWUGO, NULL, &tlsc6x_proc_ops, (void *)client);
    if (NULL == tlsc6x_proc_entry) {
        TLSC_ERROR(&client->dev, "create apk-debug entry!\n");
        return -ENOMEM;
    }
    return 0;
}

void tlsc6x_release_apk_debug_channel(void)
{
    if(tlsc6x_proc_entry){
        remove_proc_entry("tlsc6x-debug", NULL);
    }
}
#endif

#ifdef TLSC_ESD_HELPER_EN
static int esd_checker_handler(void *unused)
{
    u8 test_val;
    int ret = -1;
    ktime_t ktime;
    ////int retval = 0;

    do{
        wait_event_interruptible(tpd_esd_waiter, tpd_esd_flag != 0);
        tpd_esd_flag = 0;

        ktime = ktime_set(4, 0);
        hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
        if(auto_upd_busy){
            continue;
        }
#endif	
        if(suspend_flags	){
            continue;
        }
        //mutex_lock(&i2c_access);
        ret = tlsc6x_read_reg(this_client, 0x0,&test_val);
        //mutex_unlock(&i2c_access);
        if(ret < 0){    //maybe confused by some noise,so retry is make sense.
            msleep(30);
            //mutex_lock(&i2c_access);
            ret = tlsc6x_read_reg(this_client, 0x0,&test_val);
            //mutex_unlock(&i2c_access);
            if(ret < 0){
                // re-power-on
                ////if(g_tp_drvdata->reg_vdd){
                ////    regulator_disable(g_tp_drvdata->reg_vdd);
                ////    msleep(50);
                ////    regulator_enable(g_tp_drvdata->reg_vdd);
                ////}
                tlsc6x_tpd_reset();
            }
        }
    }while(!kthread_should_stop());

    return 0;
}
	
enum hrtimer_restart tpd_esd_kthread_hrtimer_func(struct hrtimer *timer)
{
    tpd_esd_flag = 1;
    wake_up_interruptible(&tpd_esd_waiter);

    return HRTIMER_NORESTART;
}
#endif



#ifdef TP_PROXIMITY_SENSOR
static int TP_face_get_mode(void)
{
    return PROXIMITY_SWITCH;
}

static int TP_face_mode_state(void)
{
    return PROXIMITY_STATE;
}

static int TP_face_mode_switch(int on)
{
    spin_lock(&proximity_switch_lock);

    if(1 == on){
        PROXIMITY_SWITCH = 1;
        tlsc6x_write_reg(this_client,0xb0, 0x01);
    }else if(0 == on){
        PROXIMITY_SWITCH = 0;
        tlsc6x_write_reg(this_client,0xb0, 0x00);
    }else{
        spin_unlock(&proximity_switch_lock);
        return -EINVAL;
    }

//OUT:
    spin_unlock(&proximity_switch_lock);

    return 0;
}

static int tpd_ps_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int tpd_ps_release(struct inode *inode, struct file *file)
{
    return TP_face_mode_switch(0);
}

static long tpd_ps_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int flag;

    switch (cmd) {
        case LTR_IOCTL_SET_PFLAG:
            if (copy_from_user(&flag, argp, sizeof(flag))) {
                return -EFAULT;
            }
            if (flag < 0 || flag > 1) {
                return -EINVAL;
            }
            TP_face_mode_switch(flag);
            break;
        case LTR_IOCTL_GET_PFLAG:
            flag = PROXIMITY_STATE;
            if (copy_to_user(argp, &flag, sizeof(flag))) {
                return -EFAULT;
            }
            break;
	default:
            break;
    } 

    return 0;
}

static struct file_operations tpd_ps_fops = {
    .owner				= THIS_MODULE,
    .open				= tpd_ps_open,
    .release			= tpd_ps_release,
    .unlocked_ioctl		= tpd_ps_ioctl,
};
static struct miscdevice tpd_ps_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "tpd_proximity",
    .fops = &tpd_ps_fops,
};
#endif

static struct class *touchscreen_class;
struct device *tls6x_touchscreen_cmd_dev;

static ssize_t tlsc6x_tp_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned char tpver,csver, chipver;
	unsigned char vendor_name[20];

	tpver = (g_tlsc6x_cfg_ver >> 26) & 0x3f;
	csver = (g_tlsc6x_cfg_ver >> 9) & 0x7f;
		
	if (csver >= MAX_VENDOR_ID) {
		csver = 0;
	}

	strcpy(vendor_name, tlsc6x_vendor_name[csver]);
	chipver = (unsigned char)((g_tlsc6x_chip_code >> 8) & 0xf);

	if(chipver >= MAX_CHIP_ID) {
		chipver = 0;
	}

	return sprintf(buf, __stringify(%s) ":" __stringify(%s) ":0x" __stringify(%02x) "\n",
				vendor_name, tlsc6x_chip_name[chipver], tpver);
}

static DEVICE_ATTR(tp_version, 0444, tlsc6x_tp_version_show, NULL);

static u8* firmware = NULL;
static size_t firmware_len = 0;
static ssize_t tlsc6x_tp_flash_store(struct file *filep, struct kobject *kobj, struct bin_attribute *bin_attr, char *buffer, loff_t pos, size_t size)
{
    int result;

    if (firmware == NULL) {
        struct tlsc6x_updfile_header* header = (struct tlsc6x_updfile_header*)buffer;

        if (size < sizeof(struct tlsc6x_updfile_header)) {
            TLSC_ERROR("Firmware too short\n");
            result = -EPERM;
            goto error;
        }

        if (header->sig != 0x43534843) {
            TLSC_ERROR("Invalid signature. Expected 0x43534843, got 0x%x\n", header->sig);
            result = -EPERM;
            goto error;
        }
        TLSC_INFO("Signature matched...\n");

        firmware_len =
            sizeof(struct tlsc6x_updfile_header)
            + (header->n_match * 4)
            + header->len_boot
            + header->len_cfg;

        TLSC_INFO("N_match length is %d bytes...\n", (header->n_match * 4));
        TLSC_INFO("Boot length is %d bytes...\n", header->len_boot);
        TLSC_INFO("Config length is %d bytes...\n", header->len_cfg);
        TLSC_INFO("Expected firmware length is %ld bytes...\n", firmware_len);

        firmware = kzalloc(firmware_len, GFP_KERNEL);
    }

    if (firmware == NULL || pos > firmware_len) {
        TLSC_ERROR("No memory!\n");
        result = -ENOMEM;
        goto error;
    }

    memcpy(firmware + pos, buffer, size);
    TLSC_INFO("Loaded %lld bytes...\n", pos + size);

    if (pos + size == firmware_len) {
        TLSC_INFO("Flashing touch IC firmware...\n");

        result = tlsc6x_flash_firmware(firmware, firmware_len);
        if (result != 0)
            goto error;

        tlsc6x_tpd_reset();

        TLSC_INFO("Done flashing touch IC firmware!\n");
        firmware_len = 0;
        kfree(firmware);
        firmware = NULL;
    }

    return size;

error:
    TLSC_ERROR("Error flashing firmware!\n");
    if (firmware_len)
        firmware_len = 0;
    if (firmware != NULL) {
        kfree(firmware);
        firmware = NULL;
    }
    return result;
}

static BIN_ATTR(tp_flash, 0200, NULL, tlsc6x_tp_flash_store, 0);

static int tlsc6x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
#if defined(CONFIG_ADF) || defined(XCONFIG_FB)
	int ret;
#endif
    struct input_dev *input_dev;
    struct tlsc6x_platform_data *pdata = NULL;
#ifdef TP_PROXIMITY_SENSOR
    struct input_dev *ps_input_dev;
#endif

    TLSC_INFO("%s: probe !!!++\n",__func__);

    if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
        err = -ENODEV;
        goto exit_alloc_platform_data_failed;
    }

#ifdef CONFIG_OF    // NOTE:
    if(client->dev.of_node){
        pdata = tlsc6x_parse_dt(&client->dev);
        if(pdata){
            client->dev.platform_data = pdata;
        }
    }
#endif

    if(NULL == pdata){
        err = -ENOMEM;
        TLSC_ERROR("%s: no platform data!!!\n",__func__);
        goto exit_alloc_platform_data_failed;
    }

    g_tp_drvdata = kzalloc(sizeof(*g_tp_drvdata), GFP_KERNEL);
    if(!g_tp_drvdata){
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    INIT_WORK(&g_tp_drvdata->work, tlsc6x_work_func);
    spin_lock_init(&g_tp_drvdata->irq_lock);

    this_client = client;
    g_tp_drvdata->client = client;
    g_tp_drvdata->platform_data = pdata;

    err = tlsc6x_hw_init(g_tp_drvdata);
    if(err < 0){
        goto exit_gpio_request_failed;
    }

    i2c_set_clientdata(client, g_tp_drvdata);

    g_is_telink_comp = tlsc6x_tp_dect(client);
    if(g_is_telink_comp){
        #ifdef TLSC_FORCE_UPGRADE
        tlsc6x_data_crash_deal();
        tlsc6x_tpd_reset();
        #endif
        #ifdef TLSC_AUTO_UPGRADE
        tlsc6x_auto_upgrade_buidin();
        #endif
        tlsc6x_tpd_reset();
    }else{
        TLSC_ERROR("%s, no tlsc6x!\n", __func__);
        err = -ENODEV;
        goto exit_chip_check_failed;
    }

    client->irq = gpio_to_irq(pdata->irq_gpio_number);
	
    g_tp_drvdata->suspended = false;
    INIT_WORK(&g_tp_drvdata->resume_work, tlsc6x_resume_work);
    g_tp_drvdata->tp_resume_workqueue = create_singlethread_workqueue("tlsc6x_resume_work");
    if(!g_tp_drvdata->tp_resume_workqueue) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }

    input_dev = input_allocate_device();
    if(!input_dev){
        err = -ENOMEM;
        TLSC_ERROR("failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }
    g_tp_drvdata->input_dev = input_dev;
    input_dev->name = "tlsc6x_dbg";//pClient->name;//"tlsc6x";//
    input_dev->phys = "I2C";
    input_dev->dev.parent = &client->dev;
    input_dev->id.bustype = BUS_I2C;
#ifdef TP_PROXIMITY_SENSOR
    ps_input_dev = input_allocate_device();
    if(!ps_input_dev){
        err = -ENOMEM;
        TLSC_ERROR("failed to allocate ps-input device\n");
        goto exit_input_register_device_failed;
    }
    g_tp_drvdata->ps_input_dev = ps_input_dev;
    ps_input_dev->name = "alps_pxy";
    set_bit(EV_ABS, ps_input_dev->evbit);
    input_set_capability(ps_input_dev, EV_ABS, ABS_DISTANCE); 
    input_set_abs_params(ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
    err = input_register_device(ps_input_dev);
    if (err) {
        TLSC_ERROR("failed to register input device: %s\n",
            dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

    err = misc_register(&tpd_ps_device);
    if(err){
    	TLSC_ERROR("%s: tlsc6x_ps_device register failed\n", __func__);
        goto exit_input_register_device_failed;
    }
    spin_lock_init(&proximity_switch_lock);
    spin_lock_init(&proximity_state_lock);
#endif

    __set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
    __set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    __set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
    __set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
    __set_bit(ABS_MT_PRESSURE, input_dev->absbit);

    #ifdef TOUCH_VIRTUAL_KEYS
    __set_bit(KEY_APPSELECT,  input_dev->keybit);
    __set_bit(KEY_BACK,  input_dev->keybit);
    __set_bit(KEY_HOMEPAGE,  input_dev->keybit);
    #endif
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);

    #if MULTI_PROTOCOL_TYPE_B
    input_mt_init_slots(input_dev, TS_MAX_FINGER,0);
    #endif
    input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(input_dev,ABS_MT_PRESSURE, 0, 127, 0, 0);

    set_bit(EV_ABS, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_SYN, input_dev->evbit);

    err = input_register_device(input_dev);
    if(err){
        TLSC_ERROR("failed to register input device: %s\n",dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

    #ifdef TOUCH_VIRTUAL_KEYS
    tlsc6x_virtual_keys_init();
    #endif

	#if defined(CONFIG_ADF)
	g_tp_drvdata->fb_notif.notifier_call = ts_adf_event_handler;
    g_tp_drvdata->fb_notif.priority = 1000;
    ret = adf_register_client(&g_tp_drvdata->fb_notif);
    if (ret) {
        TLSC_ERROR("[FB]unable to register fb_notifier: %d", ret);
    }
    #elif defined(XCONFIG_FB)
    g_tp_drvdata->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&g_tp_drvdata->fb_notif);
    if (ret) {
        TLSC_ERROR("[FB]Unable to register fb_notifier: %d", ret);
    }
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
	g_tp_drvdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_tp_drvdata->early_suspend.suspend = tlsc6x_ts_suspend;
	g_tp_drvdata->early_suspend.resume  = tlsc6x_ts_resume;
	register_early_suspend(&g_tp_drvdata->early_suspend);
    #endif

    #ifdef TLSC_APK_DEBUG
    tlsc6x_create_apk_debug_channel(client);
    #endif

    #ifdef TLSC_ESD_HELPER_EN
    {    // esd issue: i2c monitor thread
        ktime_t ktime = ktime_set(30, 0);
        hrtimer_init(&tpd_esd_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        tpd_esd_kthread_timer.function = tpd_esd_kthread_hrtimer_func;
        hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);	
        kthread_run(esd_checker_handler, 0, "tlsc6x_esd_helper");
    }
    #endif

    tlsc6x_wq = create_singlethread_workqueue("tlsc6x_wq");
    if (!tlsc6x_wq) {
    	TLSC_ERROR("Create workqueue failed.");
    	err = -ENOMEM;
    	goto exit_irq_request_failed;
    }

    g_tp_drvdata->irq_disabled = false;

    if (pdata->use_polling == 0) {
    	err = request_irq(client->irq, tlsc6x_interrupt,
    			IRQF_TRIGGER_FALLING, client->name, g_tp_drvdata);

    	if(err < 0){
    		TLSC_ERROR("request irq failed %d\n",err);
    		goto exit_irq_request_failed;
    	}
    } else {
        hrtimer_init(&g_tp_drvdata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        g_tp_drvdata->timer.function = tlsc6x_timer_handler;
        hrtimer_start(&g_tp_drvdata->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        TLSC_INFO("Using polling mode\n");
    }

    touchscreen_class = class_create(THIS_MODULE, "touchscreen");
	if (IS_ERR_OR_NULL(touchscreen_class)) {
		TLSC_ERROR("%s: create class error!\n", __func__);
		return -ENOMEM;
	}
	tls6x_touchscreen_cmd_dev = device_create(touchscreen_class, NULL, 0, NULL, "device");
	if (IS_ERR(tls6x_touchscreen_cmd_dev)) {
		TLSC_ERROR("Failed to create device(firmware_cmd_dev)!\n");
	}
	// version
	if (device_create_file(tls6x_touchscreen_cmd_dev, &dev_attr_tp_version) < 0) {
			TLSC_ERROR("Failed to create device file(%s)!\n", dev_attr_tp_version.attr.name);
	}

    //flash
    TLSC_INFO("Creating firmware upload device file...\n");
    if (device_create_bin_file(tls6x_touchscreen_cmd_dev, &bin_attr_tp_flash) < 0) {
        TLSC_ERROR("Failed to create firmware upload device file!\n");
    }
    TLSC_INFO("Done creating firmware upload device file...\n");

	TLSC_INFO("%s: end of probe \n",__func__);

    return 0;

exit_irq_request_failed:
    input_unregister_device(input_dev);
exit_input_register_device_failed:
    input_free_device(input_dev);
    #ifdef TP_PROXIMITY_SENSOR
    if(ps_input_dev){
        input_free_device(ps_input_dev);
    }
    #endif
exit_input_dev_alloc_failed:
exit_create_singlethread:
exit_chip_check_failed:
    gpio_free(pdata->irq_gpio_number);
    gpio_free(pdata->reset_gpio_number);
exit_gpio_request_failed:
    kfree(g_tp_drvdata);
exit_alloc_data_failed:
    g_tp_drvdata = NULL;
    i2c_set_clientdata(client, g_tp_drvdata);
exit_alloc_platform_data_failed:
    return err;
}

static int tlsc6x_remove(struct i2c_client *client)
{
    struct tlsc6x_data *drvdata = i2c_get_clientdata(client);

    TLSC_INFO("%s ++!\n", __func__);

    #ifdef TLSC_APK_DEBUG
    tlsc6x_release_apk_debug_channel();
    #endif

    #ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
    #endif
	
	#if defined(CONFIG_ADF)
    adf_unregister_client(&g_tp_drvdata->fb_notif);
    #elif defined(XCONFIG_FB)
    fb_unregister_client(&g_tp_drvdata->fb_notif);
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&drvdata->early_suspend);
    #endif

    free_irq(client->irq, drvdata);
    input_unregister_device(drvdata->input_dev);
    input_free_device(drvdata->input_dev);
	destroy_workqueue(tlsc6x_wq);

    #ifdef CONFIG_HAS_EARLYSUSPEND
    cancel_work_sync(&drvdata->resume_work);
    destroy_workqueue(drvdata->tp_resume_workqueue);
    #endif
    kfree(drvdata);
    drvdata = NULL;
    i2c_set_clientdata(client, drvdata);

    return 0;
}

static const struct i2c_device_id tlsc6x_id[] = {
    { TS_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, tlsc6x_id);

static const struct of_device_id tlsc6x_of_match[] = {
    { .compatible = "tlsc6x,tlsc6x_ts", },
    { }
};
MODULE_DEVICE_TABLE(of, tlsc6x_of_match);
static struct i2c_driver tlsc6x_driver = {
    .probe		= tlsc6x_probe,
    .remove	= tlsc6x_remove,
    .id_table	= tlsc6x_id,
    .driver	= {
        .name	= TS_NAME,
        .owner	= THIS_MODULE,
        .of_match_table = tlsc6x_of_match,
    },
};

static int __init tlsc6x_init(void)
{
	TLSC_INFO("%s: ++\n",__func__);
    return i2c_add_driver(&tlsc6x_driver);
}

static void __exit tlsc6x_exit(void)
{
    i2c_del_driver(&tlsc6x_driver);
}

late_initcall(tlsc6x_init);
module_exit(tlsc6x_exit);


MODULE_DESCRIPTION("tlsc6x touchscreen driver");
MODULE_LICENSE("GPL");
