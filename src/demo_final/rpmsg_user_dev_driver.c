#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/errno.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_config.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/delay.h>
struct _rpmsg_dev_params {
	int rpmsg_major;
	int rpmsg_minor;
	struct device *rpmsg_dev;
	struct cdev cdev;
	struct rpmsg_channel *rpmsg_chnl;
};

struct _rpmsg_dev_instance {
	struct rpmsg_endpoint *ept;
	struct _rpmsg_dev_params *dev_params;
	wait_queue_head_t usr_wait_q;
	struct mutex sync_lock;
};

#define MAX_RPMSG_BUFF_SIZE		512
#define RPMSG_KFIFO_SIZE		(MAX_RPMSG_BUFF_SIZE * 4)

#define IOCTL_CMD_GET_KFIFO_SIZE	1
#define IOCTL_CMD_GET_AVAIL_DATA_SIZE	2
#define IOCTL_CMD_GET_FREE_BUFF_SIZE	3
static struct class *rpmsg_class;//设备类
static dev_t rpmsg_dev_region;//设备号
static struct _rpmsg_dev_instance *rpmsg_dev_instance;
static struct _rpmsg_dev_params *rpmsg_dev_params;
static struct kfifo rpmsg_kfifo;
static const char init_msg[] = "init_msg";//用于通知remote端创建endpoint，内容可任意
static int block_flag = 0;
static void rpmsg_rx_cb(struct rpmsg_channel *rpdev,void *data, int len, void *priv, u32 src)
{
	//获取mutex，避免在往FIFO里写数据的同时，有任务从FIFO往外取数据，造成数据不完整或错误
		mutex_lock_interruptible(&rpmsg_dev_instance->sync_lock);
		kfifo_in(&rpmsg_kfifo, data, (unsigned int)len);//将数据放入缓冲队列中
		mutex_unlock(&rpmsg_dev_instance->sync_lock);
		//唤醒等待数据的地方
		block_flag = 1;
		wake_up_interruptible(&rpmsg_dev_instance->usr_wait_q);
}
static int rpmsg_dev_open(struct inode *inode, struct file *p_file)
{
	int retval;
	if(!(rpmsg_dev_params->rpmsg_chnl)){//rpmsg_dev_params->rpmsg_chnl为NULL说明通道还未建立
		/*向remote发送15号中断，也即virtio_info->interrupt_vring0，见virtio_device_driver.c
		通知remote端发送名称服务公告*/
		gic_raise_softirq_amp(1,15);
		mdelay(1000);//等待1秒，保证名称服务公告被处理完毕
		if(!(rpmsg_dev_params->rpmsg_chnl)){//如若仍然未接收到ns(名称服务公告)，则直接退出
				pr_err("\r\n rpmsg is not ready,try again later \r\n");	
				return -1;			
		}
	}
	//分配环形缓冲器kfifo
	retval = kfifo_alloc(&rpmsg_kfifo, RPMSG_KFIFO_SIZE, GFP_KERNEL);
	if (retval) {
		pr_err("\r\n error in kfifo_alloc for rpmsg \r\n");
		return retval;
	}
	//初始化环形缓冲器kfifo
	kfifo_reset(&rpmsg_kfifo);
	/* 初始化mutex */
	mutex_init(&rpmsg_dev_instance->sync_lock);
	/* Initialize wait queue head that provides blocking rx for userspace */
	init_waitqueue_head(&rpmsg_dev_instance->usr_wait_q);
	p_file->private_data = rpmsg_dev_instance;
	return 0;
}
static ssize_t rpmsg_dev_write(struct file *p_file,
				const char __user *ubuff, size_t len,
				loff_t *p_off)
{
	struct _rpmsg_dev_instance *dev_instance = p_file->private_data;
	struct _rpmsg_dev_params *dev_params = dev_instance->dev_params;
	int err;
	unsigned int size;
	char *tmp_buff;

	if (len < MAX_RPMSG_BUFF_SIZE)
		size = len;
	else
		size = MAX_RPMSG_BUFF_SIZE;

	tmp_buff = kzalloc(size, GFP_KERNEL);

	if (copy_from_user(tmp_buff, ubuff, size)) {
		pr_err("\r\n user to kernel buff copy error \r\n");
		return -1;
	}

	err = rpmsg_send_offchannel(dev_params->rpmsg_chnl,
					dev_instance->ept->addr,
					dev_params->rpmsg_chnl->dst,
					tmp_buff, size);

	if (err) {
		size = 0;
		pr_err("\r\n rpmsg_send_off_channel error \r\n");
	}

	return size;
}
static ssize_t rpmsg_dev_read(struct file *p_file, char __user *ubuff,size_t len, loff_t *p_off)
{
	struct _rpmsg_dev_instance *dev_instance = p_file->private_data;
	int retval;
	unsigned int data_available, data_used, bytes_copied;
	//获取mutex，避免在获取FIFO数据的同时，有数据被写入FIFO，造成混乱。
	if (mutex_lock_interruptible(&dev_instance->sync_lock))
		return -ERESTARTSYS;

	data_available = kfifo_len(&rpmsg_kfifo);

	if (data_available ==  0) {
		mutex_unlock(&dev_instance->sync_lock);
		/* if non-blocking read is requested return error */
		if (p_file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		/* Block the calling context till data becomes available */
		wait_event_interruptible(dev_instance->usr_wait_q,
					block_flag != 0);
	}

	/* reset block flag */
	block_flag = 0;
	/* Provide requested data size to user space */
	data_available = kfifo_len(&rpmsg_kfifo);
	data_used = (data_available > len) ? len : data_available;
	//把数据从fifo复制到用户空间当中
	retval = kfifo_to_user(&rpmsg_kfifo, ubuff, data_used, &bytes_copied);

	/* Release lock on rpmsg kfifo */
	mutex_unlock(&dev_instance->sync_lock);

	return retval ? retval : bytes_copied;
}
static long rpmsg_dev_ioctl(struct file *p_file, unsigned int cmd,unsigned long arg)
{
	unsigned int tmp;

	switch (cmd) {
	case IOCTL_CMD_GET_KFIFO_SIZE:
		tmp = kfifo_size(&rpmsg_kfifo);
		if (copy_to_user((unsigned int *)arg, &tmp, sizeof(int)))
			return -EACCES;
		break;

	case IOCTL_CMD_GET_AVAIL_DATA_SIZE:
		tmp = kfifo_len(&rpmsg_kfifo);
		pr_err("kfifo len ioctl = %d ", kfifo_len(&rpmsg_kfifo));
		if (copy_to_user((unsigned int *)arg, &tmp, sizeof(int)))
			return -EACCES;
		break;
	case IOCTL_CMD_GET_FREE_BUFF_SIZE:
		tmp = kfifo_avail(&rpmsg_kfifo);
		if (copy_to_user((unsigned int *)arg, &tmp, sizeof(int)))
			return -EACCES;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}
static void rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
//暂时保留，未使用
}
static int rpmsg_dev_release(struct inode *inode, struct file *p_file)
{
	struct _rpmsg_dev_instance *rpmsg_dev_instance = p_file->private_data;
	kfifo_free(&rpmsg_kfifo);
	return 0;
}
static const struct file_operations rpmsg_dev_fops = {
	.owner = THIS_MODULE,
	.read = rpmsg_dev_read,
	.write = rpmsg_dev_write,
	.open = rpmsg_dev_open,
	.unlocked_ioctl = rpmsg_dev_ioctl,
	.release = rpmsg_dev_release,
};
static int rpmsg_user_dev_drv_probe(struct rpmsg_channel *rpdev){
//当接收到remote发来的名称服务公告并完成rpmsg_channel(看成一个设备)的创建后
//(设备名为rpmsg-openamp-demo-channel)，此时将执行该probe函数。
//在这边主要完成建立通信通道上master端的endpoint用于核间的通信
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);
		
/*====================创建endpoint=================================
绑定相应的callback为rpmsg_rx_cb，当master接收到接收中断时会
根据接收到的信息(信息中包括数据，以及消息源地址和目标地址)
匹配对应的endpoint，并调用对应的回调函数*/
	rpmsg_dev_instance->ept = rpmsg_create_ept(rpdev,rpmsg_rx_cb, rpmsg_dev_instance, RPMSG_ADDR_ANY);
	if(rpmsg_dev_instance->ept == NULL){
		pr_err(" Endpoint creation for failed!\r\n");
		return -ENOMEM;		
	}
	/*给remote端发送消息，这是第一次给remote端发消息，remote端接收到数据后只进行endpoint创建，
	调用对应的created_channel调用(见remote 运用程序)，而不对数据进行任何处理，
	//同时与master端的endpoint完成对接。所发送的消息包括master endpoint的源地址和数据*/
	int retval;
	retval = rpmsg_send_offchannel(rpdev,
					rpmsg_dev_instance->ept->addr,
					rpdev->dst,
					init_msg, sizeof(init_msg));//

	if (retval) {
		pr_err(" Init message send failed!\r\n");
		return retval;
	}
	rpmsg_dev_params->rpmsg_chnl = rpdev;
//	dev_set_drvdata(&rpdev->dev,rpmsg_dev_instance);
	return 0;
}
static void rpmsg_user_dev_drv_remove(struct rpmsg_channel *rpdev)
{					
 	rpmsg_destroy_ept(rpmsg_dev_instance->ept);
}
static struct rpmsg_device_id rpmsg_user_dev_drv_id_table[] = {
	{ .name	= "rpmsg-openamp-demo-channel" },//该名称应与remote端的名称一致
	{ },									//remote通过名称服务公告告知master
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_user_dev_drv_id_table);

static struct rpmsg_driver rpmsg_user_dev_drv = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_user_dev_drv_id_table,
	.probe		= rpmsg_user_dev_drv_probe,
	.callback	= rpmsg_cb,
	.remove	= rpmsg_user_dev_drv_remove,
};
static int __init init(void){
/*当该模块被加载时，进入该init函数，*/
	rpmsg_dev_params = kzalloc(sizeof(struct _rpmsg_dev_params),GFP_KERNEL);
	if(!rpmsg_dev_params){
		pr_err("\r\n cannot allocate memory for rpmsg device params \r\n");
		return -ENOMEM;
	}
	rpmsg_dev_instance = kzalloc(sizeof(struct _rpmsg_dev_instance),GFP_KERNEL);
	if (!rpmsg_dev_instance) {
		pr_err("\r\n cannot allocate memory for rpmsg device instance \r\n");
		return -ENOMEM;
	}
	rpmsg_dev_instance->dev_params = rpmsg_dev_params;
//========================首先注册字符设备================================
	if (alloc_chrdev_region(&rpmsg_dev_region, 0/*起始设备号*/, 1/*次设备号个数*/, KBUILD_MODNAME) < 0) {//动态分配设备号
		pr_err("\r\n Error allocating char device \r\n");
		return -1;
	}
	rpmsg_dev_params->rpmsg_major = MAJOR(rpmsg_dev_region);//获取主设备号
	rpmsg_dev_params->rpmsg_minor = MINOR(rpmsg_dev_region);//获取次设备号
	rpmsg_dev_params->rpmsg_chnl = NULL;
	rpmsg_class = class_create(THIS_MODULE,KBUILD_MODNAME);
	if(NULL == rpmsg_class){//创建设备类
		unregister_chrdev_region(rpmsg_dev_region, 1);
		pr_err("\r\n Error allocating char device \r\n");
		return -1;
	}
	//创建设备节点，属于rpmsg_class设备类，名为rpmsg，将存在于/dev目录下;
	//如果不创建该节点，直接使用cdev_init,cdev_add则只在/proc/devices下生成一个设备号为rpmsg_dev_region的设备
	//不能用open打开。需要通过mknod进行手动创建节点后才可以使用
	cdev_init(&rpmsg_dev_params->cdev, &rpmsg_dev_fops);//初始化字符设备
	rpmsg_dev_params->cdev.owner = THIS_MODULE;
	//添加字符设备，进行这一部分之前应完成设备号的申请
	if (cdev_add(&rpmsg_dev_params->cdev, rpmsg_dev_region, 1) == -1) {
		device_destroy(rpmsg_class, rpmsg_dev_region);
		class_destroy(rpmsg_class);
		unregister_chrdev_region(rpmsg_dev_region, 1);
		return -1;
	}
	rpmsg_dev_params->rpmsg_dev = device_create(rpmsg_class, NULL,rpmsg_dev_region, NULL, "rpmsg");
	if (rpmsg_dev_params->rpmsg_dev == NULL) {
		class_destroy(rpmsg_class);
		unregister_chrdev_region(rpmsg_dev_region, 1);
		return -1;
	}
//======================设备创建结束=====================================
/*执行rpmsg_driver的注册，一旦名为rpmsg-openamp-demo-channel
的设备被注册且存在于rpmsg总线上，会先后执行对应的bus->probe(见virtio_rpmsg_bus.c中的rpmsg_dev_probe函数)
和drv->probe(即rpmsg_user_dev_drv_probe)*/
	return register_rpmsg_driver(&rpmsg_user_dev_drv);
}

static void __exit fini(void){
	unregister_rpmsg_driver(&rpmsg_user_dev_drv);//卸载rpmsg driver
	cdev_del(&rpmsg_dev_params->cdev);//删除字符设备
	device_destroy(rpmsg_class, rpmsg_dev_region);//删除设备节点
	unregister_chrdev_region(rpmsg_dev_region, 1);//释放占用的设备号
	kfree(rpmsg_dev_params);
	kfree(rpmsg_dev_instance);
	
}
module_init(init);
module_exit(fini);

MODULE_DESCRIPTION("Sample driver to exposes rpmsg svcs to userspace via a char device");
MODULE_LICENSE("GPL v2");
