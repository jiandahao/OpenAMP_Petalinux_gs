#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of_irq.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_config.h>
#include <asm/outercache.h>
#include <asm/cacheflush.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/idr.h>
//#include <linux/irqchip/arm-gic.h>
#include <linux/irqchip/arm-gic.h>
//#include <linux/smp.h>//add header , which include set_ipi_handler

static DEFINE_IDA(dev_index);
#define RPMSG_NUM_VRINGS 2
// #define MEMSTART 0x08000000
// #define MEMEND   0x10000000
// #define NUMDESCS 256
#define VIRTIO_ID 7
struct rpmsg_vring {//记录每一个VRing的信息

	void	   *va;
	dma_addr_t dma;
	int		num_descs;
	u32		da;
	u32		align;
	struct virtqueue *vq;
};
struct virtio_instance {
	unsigned int  mem_start;
	unsigned int  mem_end;
	unsigned int  num_descs;
	unsigned long dev_feature;
	// unsigned int  tx_vring;
	// unsigned int  rx_vring;
	unsigned int  vring_align;
	unsigned int  num_vrings;
	unsigned int  interrupt_vring0;
	unsigned int  interrupt_vring1;
	//unsigned int  gen_feature;//记录feature 以便在remote processor启动后通知它
	unsigned int  status;
	struct 		  virtio_device virtio_dev;
	struct 		  rpmsg_vring vrings[RPMSG_NUM_VRINGS];
	struct device mid_dev;
};

struct virtio_instance *virtio_info;
struct  platform_device *rpmsg_platform;
static struct work_struct workqueue;
//static struct work_struct workqueue_vring1;
static void ipi_kick(void)
{//使用工作队列的目的是为了保证中断处理函数处理时间足够短。将处理过程交由系统去调度执行
	//dev_dbg(&rpmsg_platform->dev, "KICK Linux because of pending message\n");
	printk(KERN_INFO "\r\nMaster:IRQ Received,kick remote");
	schedule_work(&workqueue);
}
static irqreturn_t handle_event(struct work_struct *work)
{

	flush_cache_all();
	outer_flush_range(virtio_info->mem_start, virtio_info->mem_end);
	if (vring_interrupt(0, virtio_info->vrings[0].vq) == IRQ_NONE)
		dev_dbg(&rpmsg_platform->dev, "no message found in vqid 0\n");
	printk(KERN_INFO "\r\nMaster:IRQ Received,handle_event");
	return IRQ_HANDLED;
}

static bool rpmsg_virtio_notify(struct virtqueue *vq)
{
	/* Notify the other core. */
	if (vq == virtio_info->vrings[0].vq)
		/* Raise soft IRQ on GIC. */
		//send interrupt 15 to let remote know that it is ready to communicate
		gic_raise_softirq_amp(1, virtio_info->interrupt_vring0);	
	else
		//send interrupt 14 to notify the remote to receive message
		gic_raise_softirq_amp(1, virtio_info->interrupt_vring1);
	return true;
}

static void rpmsg_virtio_del_vqs(struct virtio_device *vdev){
	/* 暂确认可行*/
		int i;
		struct rpmsg_vring *local_vring ;
		for (i = 0; i < RPMSG_NUM_VRINGS; i++) {
			local_vring = &(virtio_info->vrings[i]);
			vring_del_virtqueue(local_vring->vq);
			local_vring->vq = NULL;
			dma_free_coherent(&(rpmsg_platform->dev),
					local_vring->num_descs, local_vring->va,
					local_vring->dma);
		}
}
static int rpmsg_virtio_find_vqs(struct virtio_device *vdev,
					unsigned nvqs, struct virtqueue *vqs[],
					vq_callback_t *callbacks[],
					const char *names[])
{
		int i,size;
		struct rpmsg_vring *local_vring;
		for(i = 0;i < nvqs ; i++)//nvqs值在virtio driver的probe函数中指定为2
		{
			local_vring = &(virtio_info->vrings[i]);
			//计算vring的大小
			size = vring_size(virtio_info->num_descs,virtio_info->vring_align);
			//为vring分配连续物理内存
			/* Allocate non-cacheable memory for the vring. */
			local_vring->num_descs = virtio_info->num_descs;
			local_vring->align = virtio_info->vring_align;
			local_vring->va = dma_alloc_coherent(&(rpmsg_platform->dev),size,&(local_vring->dma),GFP_KERNEL);
			memset(local_vring->va,0,size);//完成初始化
			local_vring->vq = vring_new_virtqueue(i,
						virtio_info->num_descs,
						virtio_info->vring_align, vdev,
						false, local_vring->va ,
						rpmsg_virtio_notify,//notify 函数,用于通知remote processor,TODO:
						callbacks[i], names[i]);//name和callback在virtio驱动中指定
			vqs[i] = local_vring->vq;	
		//	dev_info(&(vdev), "dma: 0x%X va:0x%X:\n", local_vring->dma, local_vring->va);
			//dev_info(&(vdev), "dma: 0x%X va:\n", &(local_vring->dma), &(local_vring->va));
			printk(KERN_INFO "\r\nMaster:vring[%d][0:rsq,1:svq] dma: 0x%X va:0x%X \n",i,local_vring->dma,local_vring->va);
		}
		return 0;
}

static u64 rpmsg_virtio_get_features(struct virtio_device *vdev)
{
	return virtio_info->dev_feature;
}
static int rpmsg_virtio_finalize_features(struct virtio_device *vdev)
{
	/* Set vring transport features. */
	vring_transport_features(vdev);
	/* Make sure we don't have any features > 32 bits! */
	BUG_ON((u32)vdev->features != vdev->features);
	/*
	 * Remember the finalized features of our vdev, and provide it
	 * to the remote processor once it is powered on.
	 */
	virtio_info->gen_feature = vdev->features;//保存features
	return 0;
}
static u8 rpmsg_virtio_get_status(struct virtio_device *vdev, u8 status)
{
   
	return virtio_info->status;
}
static void rpmsg_virtio_set_status(struct virtio_device *vdev, u8 status){
	
	virtio_info->status = status;
}
static void rpmsg_virtio_reset(struct virtio_device *vdev)
{	/* */
	virtio_info->status = 0;
}
static const struct virtio_config_ops rpmsg_virtio_config_ops = {
	.get_features	= rpmsg_virtio_get_features,
	.finalize_features = rpmsg_virtio_finalize_features,
	.find_vqs	= rpmsg_virtio_find_vqs,//virtio device和virtio driver匹配成功后会调用，，，within rpmsg_probe
	.del_vqs	= rpmsg_virtio_del_vqs,
	.reset		= rpmsg_virtio_reset,
	//virtio device和virtio driver匹配成功后会调用,由kick时调用，，，within rpmsg_probe
	.set_status	= rpmsg_virtio_set_status,
	.get_status	= rpmsg_virtio_get_status,//设置status初始值为0，在kick之前会检查设备是否准备好
	//此时调用get_status获取status，并与VIRTIO_CONFIG_S_DRIVER_OK(为4)比较，如果不相等(正常情况下
	//都不应该相等则set_status)为VIRTIO_CONFIG_S_DRIVER_OK
};
	
static int get_devinfo_from_dts(struct platform_device *pdev){
	return 0;
}
static struct device_type mid_level_type = {
	.name		= "rpmsg_mid",
	//.release	= mid_level_type_release,
};
static int rpmsg_virtio_probe(struct platform_device *pdev){
	int ret;
	int index;
	struct virtio_device *virtio_dev;

	rpmsg_platform = pdev;//记录pdev
	virtio_info  = kzalloc(sizeof(struct virtio_instance), GFP_KERNEL);
	if (!virtio_info) {
		dev_err(&pdev->dev, "Unable to alloc memory for virtio_instance.\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, virtio_info);

	virtio_info->mem_start = 0x08000000;
	virtio_info->mem_end = 0x10000000;
	virtio_info->num_descs = 256;
	virtio_info->vring_align = 0x1000;
	virtio_info->interrupt_vring0 = 15;
	virtio_info->interrupt_vring1 = 14;
	virtio_info->num_vrings = 2;
	virtio_info->dev_feature = 0x00000001;
	virtio_info->gen_feature = 0x0;
	virtio_info->status = 0;


	ret = get_devinfo_from_dts(pdev);//leave it empty now! TODO:从设备树读取virtio设备预定义信息。
	if( 0 != ret){
		dev_err(&pdev->dev,"failed to get information from dts\n");
		return -ENODEV;
	}
	ret = dma_declare_coherent_memory(&pdev->dev,
					virtio_info->mem_start,
					virtio_info->mem_start,
					virtio_info->mem_end -
					virtio_info->mem_start + 1,
					DMA_MEMORY_IO);

	if (!ret) {
		dev_err(&pdev->dev, "dma_declare_coherent_memory failed\n");
		return -ENODEV;
	}
	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		return -ENODEV;
	}
	INIT_WORK(&workqueue,handle_event);
	
	set_ipi_handler(virtio_info->interrupt_vring0,ipi_kick, "Firmware kick");

	/*由于在virtio_rpmsg_bus.c中有
	bufs_va = dma_alloc_coherent(vdev->dev.parent->parent,
				     total_buf_space, &vrp->bufs_dma,GFP_KERNEL);
				     vdev->dev.parent->parent对应的应该是pdev->dev,
	所以这里创建一个中间设备只是一种弥补手段，该设备没有实际意义*/
	device_initialize(&(virtio_info->mid_dev));

	virtio_info->mid_dev.parent = &(pdev->dev);
	virtio_info->mid_dev.type = &mid_level_type;

	index = ida_simple_get(&dev_index, 0, 0, GFP_KERNEL);

	if (index < 0) {
		put_device(&(virtio_info->mid_dev));
		return -ENODEV;
	}

	dev_set_name(&(virtio_info->mid_dev), "rpmsg_mid%d", index);

	device_add(&(virtio_info->mid_dev));
	/*中间设备添加到此结束*/

	//注册virtio设备
	virtio_dev = &(virtio_info->virtio_dev);
	virtio_dev->config = &rpmsg_virtio_config_ops;
	virtio_dev->dev.parent = &(virtio_info->mid_dev);
	virtio_dev->id.device = VIRTIO_ID;//指定ID 为7，即为rpmsg设备	
	ret = register_virtio_device(virtio_dev);
	dev_info(&(pdev->dev), "registered %s (type %d)\n", dev_name(&virtio_dev->dev), virtio_dev->id.device);
	return 0;
}
static int rpmsg_virtio_remove(struct platform_device *pdev)
{
	unregister_virtio_device(&(virtio_info->virtio_dev));

	put_device(&(virtio_info->mid_dev));

	dma_release_declared_memory(&pdev->dev);

	clear_ipi_handler(virtio_info->interrupt_vring0);
	kfree(virtio_info);

	return 0;
}


/* Match table for OF platform binding */
static struct of_device_id rpmsg_virtio_match[] = {
	{ .compatible = "xlnx,rpmsg_virtio_driver", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, rpmsg_virtio_match);

static struct platform_driver rpmsg_virtio_driver = {
	.probe  = rpmsg_virtio_probe,
	.remove =rpmsg_virtio_remove,
	.driver = {
		.name = "rpmsg_virtio_driver",
		.owner = THIS_MODULE,
		.of_match_table = rpmsg_virtio_match,
	},
};

static int __init init(void)
{
	return platform_driver_register(&rpmsg_virtio_driver);
}

static void __exit fini(void)
{
	platform_driver_unregister(&rpmsg_virtio_driver);
}
module_init(init);
module_exit(fini);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Zynq RPMSG driver to use RPMSG framework without remoteproc");

