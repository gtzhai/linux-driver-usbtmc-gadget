/*
 * Gadget Driver for usbtmc dev 
 *
 * Copyright (C) 2018 gtzhai.
 * Author: gtzhai <gtzhai@163.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/usb/composite.h>
#include "tmc.h"
#include "u_tmc.h"

//#define TMC_DEBUG 1


#define TMC_BULK_BUFFER_SIZE          (10*1024+12+3) 
#define TX_REQ_MAX 1

static const char tmc_shortname[] = "usb_tmc";

/** struct to hold all usb tmc transfer relevant information */
typedef struct 
{
	uint8_t bTag; /**< contains the bTag value of the currently active transfer */
	uint32_t nbytes_rxd; /**< contains the number of bytes received in active tmc OUT transfer */
	uint32_t nbytes_txd; /**< contains the number of bytes transmitted in active tmc IN transfer */
}TMCTransferInfo;

#define USB_TMC_HEADER_SIZE 12  /**< length of a USBTMC header */

/* USBTMC MsgID. Values, Ref.: Table 2 */
#define DEV_DEP_MSG_OUT              1   /**< device dependent command message */
#define REQUEST_DEV_DEP_MSG_IN       2   /**< command message that requests the device to send a USBTMC response */
#define DEV_DEP_MSG_IN               2   /**< response message to the REQUEST_DEV_DEP_MSG_IN */
#define VENDOR_SPECIFIC_OUT          126 /**< vendor specific command message */
#define REQUEST_VENDOR_SPECIFIC_IN   127 /**< command message that requests the device to send a vendor specific USBTMC response */
#define VENDOR_SPECIFIC_IN           127 /**< response message to the REQUEST_VENDOR_SPECIFIC_IN */


/* USBTMC USB488 Subclass commands (bRequest values, Ref.: Table 9) */
#define READ_STATUS_BYTE             128
#define REN_CONTROL                  160
#define GO_TO_LOCAL                  161
#define LOCAL_LOCKOUT                162

/* bmTransfer attributes */
#define bmTA_EOM       0x01  /**< bmTransfer Attribute: End of Message */
#define bmTA_TERMCHAR  0x02  /**< bmTransfer Attribute: Terminate transfer with Terminate Character */

/* defines for the device capablilities, Ref.: Table 37 and Table 8 USB488 */
#define HAS_INDICATOR_PULSE 0x04
#define TALK_ONLY       0x02
#define LISTEN_ONLY     0x01
#define TERMCHAR_BULKIN 0x01
#define IS_488_2        0x04
#define ACCEPTS_LOCAL_LOCKOUT 0x02
#define TRIGGER         0x01
#define SCPI_COMPILIANT 0x08
#define SR1_CAPABLE     0x04
#define RL1_CAPABLE     0x02
#define DT1_CAPABLE     0x01

typedef struct 
{
	uint8_t USBTMC_status;
	uint8_t reserved0;
	uint8_t bcdUSBTMC_lsb;
	uint8_t bcdUSBTMC_msb;
	uint8_t TMCInterface;
	uint8_t TMCDevice;
	uint8_t reserved1[6];
	/* place here USB488 subclass capabilities */
	uint8_t bcdUSB488_lsb;
	uint8_t bcdUSB488_msb;
	uint8_t USB488Interface;
	uint8_t USB488Device;
	uint8_t reserved2[8];
} USB_TMC_Capabilities;

struct tmc_dev 
{
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;
	struct usb_ep *ep_int_in;

	atomic_t online;
	//atomic_t error;

	atomic_t read_excl;
	atomic_t write_excl;
	//atomic_t open_excl;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req;

	atomic_t ioctl_excl;
	struct usb_request *int_in_req;
	wait_queue_head_t int_in_wq;

	int rx_done;

	char buf[512];
	TMCTransferInfo usb_tmc_transfer; 
	int usb_tmc_status; 
	uint8_t term_char_enabled;
	uint8_t term_char;
	uint8_t usbtmc_last_write_bTag;
	uint8_t usbtmc_last_read_bTag;
};

static volatile const USB_TMC_Capabilities USB_TMC_CAPABILITIES = {
  	USBTMC_STATUS_SUCCESS,
	0,
	0x10, 0x01,/* BCD version number of TMC specification, 2.00 */
	0x00,
	0,
	{0,0,0,0,0,0},

    	/* place here USB488 subclass capabilities */
    	0x10, 0x01, /* BCD version number of USB488 specification, 2.00 */
      	0,
      	0,
	{0,0,0,0,0,0,0,0}
};


static struct usb_interface_descriptor tmc_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 3,
	.bInterfaceClass        = USB_CLASS_APP_SPEC,
	.bInterfaceSubClass     = 0x03,
	.bInterfaceProtocol     = 0,
};

static struct usb_endpoint_descriptor tmc_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_highspeed_int_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize         = __constant_cpu_to_le16(64),
	.bInterval              = 2,
};

static struct usb_endpoint_descriptor tmc_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tmc_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tmc_fullspeed_int_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_INT,
};

static struct usb_descriptor_header *fs_tmc_descs[] = {
	(struct usb_descriptor_header *) &tmc_interface_desc,
	(struct usb_descriptor_header *) &tmc_fullspeed_in_desc,
	(struct usb_descriptor_header *) &tmc_fullspeed_out_desc,
	(struct usb_descriptor_header *) &tmc_fullspeed_int_in_desc,
	NULL,
};

static struct usb_descriptor_header *hs_tmc_descs[] = {
	(struct usb_descriptor_header *) &tmc_interface_desc,
	(struct usb_descriptor_header *) &tmc_highspeed_in_desc,
	(struct usb_descriptor_header *) &tmc_highspeed_out_desc,
	(struct usb_descriptor_header *) &tmc_highspeed_int_in_desc,
	NULL,
};
static struct tmc_dev *_tmc_dev;

static inline struct tmc_dev *func_to_tmc(struct usb_function *f)
{
	return container_of(f, struct tmc_dev, function);
}


static struct usb_request *tmc_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
	{
		return NULL;
	}

	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf)
       	{
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void tmc_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) 
	{
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int tmc_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) 
	{
		return 0;
	}
       	else
       	{
		atomic_dec(excl);
		return -1;
	}
}

static inline void tmc_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

void tmc_req_put(struct tmc_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

struct usb_request *tmc_req_get(struct tmc_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) 
	{
		req = 0;
	}
       	else
       	{
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void tmc_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_dev *dev = _tmc_dev;

	if (req->status != 0) 
	{
		if (req->status != -ESHUTDOWN)
		{
#ifdef TMC_DEBUG
			printk(KERN_INFO "[USB] %s: warning (%d)\n", __func__, req->status);
#endif
		}
	}

	tmc_req_put(dev, &dev->tx_idle, req);
	wake_up(&dev->write_wq);
}

static void tmc_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_dev *dev = _tmc_dev;

	dev->rx_done = 1;
	if (req->status != 0) 
	{
		if (req->status != -ESHUTDOWN)
		{
#ifdef TMC_DEBUG
			printk(KERN_INFO "[USB] %s: warning (%d)\n", __func__, req->status);
#endif
		}
	}
	//printk("%s:receive data done1\n", __func__);
	wake_up(&dev->read_wq);
	//printk("%s:receive data done2\n", __func__);
}

static void tmc_complete_int_in(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_dev *dev = _tmc_dev;

	if (req->status != 0) 
	{
		if (req->status != -ESHUTDOWN)
		{
#ifdef TMC_DEBUG
			printk(KERN_INFO "[USB] %s: warning (%d)\n", __func__, req->status);
#endif
		}
	}

	wake_up(&dev->int_in_wq);
}


static int tmc_create_bulk_endpoints(struct tmc_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc,
				struct usb_endpoint_descriptor *int_in_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

#ifdef TMC_DEBUG
	printk("create_bulk_endpoints dev: %p\n", dev);
#endif
	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep)
       	{
		printk("usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
#ifdef TMC_DEBUG
	printk("usb_ep_autoconfig for ep_in got %s\n", ep->name);
#endif
	ep->driver_data = dev;		
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) 
	{
		printk("usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}

#ifdef TMC_DEBUG
	printk("usb_ep_autoconfig for tmc ep_out got %s\n", ep->name);
#endif
	ep->driver_data = dev;		
	dev->ep_out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, int_in_desc);
	if (!ep)
       	{
		printk("usb_ep_autoconfig for ep_int_in failed\n");
		return -ENODEV;
	}
	ep->driver_data = dev;
	dev->ep_int_in = ep;
	
	req = tmc_request_new(dev->ep_out, TMC_BULK_BUFFER_SIZE);
	if (!req)
	{
		goto fail;
	}
	req->complete = tmc_complete_out;
	dev->rx_req = req;

	for (i = 0; i < TX_REQ_MAX; i++) 
	{
		req = tmc_request_new(dev->ep_in, TMC_BULK_BUFFER_SIZE);
		if (!req)
		{
			goto fail;
		}

		req->complete = tmc_complete_in;
		tmc_req_put(dev, &dev->tx_idle, req);
	}

	req = tmc_request_new(dev->ep_int_in, 512);
	if (!req)
	{
		goto fail;
	}
	req->complete = tmc_complete_int_in;
	dev->int_in_req = req;

	return 0;

fail:
	printk(KERN_ERR "tmc_bind() could not allocate requests\n");
	return -1;
}

static int tmc_queue_req(struct usb_ep *ep,
			       struct usb_request *req, gfp_t gfp_flags)
{
	int ret = 0;
	int try_cnt = 0;

	do
	{
		ret = usb_ep_queue(ep, req, gfp_flags);
		try_cnt++;
		if(try_cnt > 3)
		{
			printk("queue is always not empty, so empty it forcely, and queue the new req\n");
			usb_ep_fifo_flush(ep);
			ret = usb_ep_queue(ep, req, gfp_flags);
			break;
		}
	}while(ret == -EINVAL);

	return ret;
}

static ssize_t tmc_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct tmc_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	unsigned long int n_characters, done = 0;
	u8 *req_buf;
	
#ifdef TMC_DEBUG
	printk("tmc_read(%d)\n", count);
#endif
	//check params
	if (!_tmc_dev)
	{
		printk("tmc_read !tmc_dev\n");
		return -ENODEV;
	}
	if (count > TMC_BULK_BUFFER_SIZE-12-3)
	{
		printk("tmc_read count > %d\n", TMC_BULK_BUFFER_SIZE-12-3);
		return -EINVAL;
	}
	if (tmc_lock(&dev->read_excl))
	{
		printk("tmc_read lock read excl wrong\n");
		return -EBUSY;
	}

	
	/*waiting for state of ready to read*/
	if(!(atomic_read(&dev->online))) 
	{
#ifdef TMC_DEBUG
		printk("tmc_read: waiting for online state\n");
#endif
		tmc_unlock(&dev->read_excl);
		return -EIO;
	}

	req  = dev->rx_req;
       	req_buf = (u8 *)req->buf;
	xfer = count;

#ifdef TMC_DEBUG
	printk("tmc_read start read\n");
#endif
	while(1)
	{
		/*read pdu*/
		dev->rx_done = 0;
		req->length = xfer+12;
#ifdef TMC_DEBUG
		printk("tmc_read: queue req %p\n", req);
#endif
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) 
		{
			printk("tmc_read: failed to queue req %p (%d)\n", req, ret);
			r = -EIO;
			break;
		}

		/*wait the request complete, if something wrong, just dequeue it*/	
		ret = wait_event_timeout(dev->read_wq, dev->rx_done, 1000);
	        //ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
		if (ret <= 0) 
		{
			if(dev->rx_done == 0)
			{
			if (ret != -ERESTARTSYS)
			{
			}

			r = ret;
			usb_ep_dequeue(dev->ep_out, req);
#ifdef TMC_DEBUG
			printk("read data always undone:%d, %d\n", ret, dev->rx_done);
#endif
			break;
			}
		}
#ifdef TMC_DEBUG
		printk("tmc_read: req is processed %d, %d\n", dev->rx_done, ret);
#endif
		/*if no error, then copy to user*/
		{
			int i;

#ifdef TMC_DEBUG
			printk("%s:rx %p %d\n", __func__, req, req->actual);

			for(i=0; i<=req->actual; i++)
			{
				printk("%s:buf[%d]=%x\n", __func__, i, req_buf[i]);	
			}
#endif

			if(req_buf[0] == DEV_DEP_MSG_OUT)	
			{
				u8 btag = req_buf[1];
				u8 btag_rev = ~req_buf[2];

				if(btag == btag_rev)	
				{
		  			dev->usb_tmc_transfer.bTag = req_buf[1];
					dev->usbtmc_last_read_bTag = req_buf[1];
					//dev->usb_tmc_transfer.bTag++;
					if(!dev->usb_tmc_transfer.bTag)
					{
						//dev->usb_tmc_transfer.bTag++;
					}

					n_characters=req_buf[4]+(req_buf[5]<<8)+(req_buf[6]<<16)+(req_buf[7]<<24);

					xfer = req->actual - 12;

					if(n_characters <= xfer)
					{
						if (copy_to_user(buf+done, req->buf+12, n_characters))
						{
							printk("copy to user wrong\n");
							r = -EFAULT;
							break;
						}

						done += n_characters;	
						if(req_buf[8] & 0x1)
						{
							//printk("the pdu read comletely\n");
							break;
						}
						else
						{
							if(done >= count)
							{
								printk("pc send too much data than wanted, the pdu can not read comletely\n");
								r = -EIO;
								break;	
							}
							else
							{
								xfer = count - done;
							}
						}
					}
					else
					{
						if (copy_to_user(buf+done, req->buf+12, xfer))
						{
							printk("copy to user wrong 2\n");
							r = -EFAULT;
							break;
						}
						r = -EIO;
						printk("pc send too much data than wanted, the pdu can not read comletely 2\n");
						break;

					}
				}
				else
				{
					printk("btag and btag rev wrong:0x%x, 0x%x\n", btag, btag_rev);	
					r = -EIO;
					break;
				}
			}
			else
			{
				printk("read not supported msg id %d\n", req_buf[0]);	
				r = -EIO;
				break;
			}
		}
	}

	tmc_unlock(&dev->read_excl);
	if(r < 0)
	{
		dev->usb_tmc_transfer.nbytes_rxd = 0;
		pr_debug("tmc_read succes returning %d\n", r);
		return r;
	}
	else
	{
		dev->usb_tmc_transfer.nbytes_rxd = done;
		pr_debug("tmc_read wrong returning %ld\n", done);
		return done;
	}
}

static ssize_t tmc_read_REQUEST_DEV_DEP_MSG_IN(struct tmc_dev *dev)
{
	struct usb_request *req;
	unsigned long int n_characters;
	int ret, r;
	u8 *req_buf;

#ifdef TMC_DEBUG
	printk("%s: in\n", __func__);
#endif

	if (tmc_lock(&dev->read_excl))
	{
		return -EBUSY;
	}

#ifdef TMC_DEBUG
	printk("%s: 1\n", __func__);
#endif
	/*send the pdu*/
	req = dev->rx_req;
       	req_buf = (u8 *)req->buf;
	req->length = 12+10240;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) 
	{
		printk("tmc_read_REQUEST_DEV_DEP_MSG_IN: read failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		tmc_unlock(&dev->read_excl);
		return r;
	}
#ifdef TMC_DEBUG
	printk("%s: 2\n", __func__);
#endif

	/*wait the request complete, if not dequeue it*/	
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
	//ret = wait_event_timeout(dev->read_wq, dev->rx_done, 1000);
	if (ret < 0) 
	{
		if (ret != -ERESTARTSYS)
		{
		}
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		tmc_unlock(&dev->read_excl);
		return r;
	}
#ifdef TMC_DEBUG
	printk("%s: 3\n", __func__);
#endif

	/*if no error, then check it*/
	{
		int i;
#ifdef TMC_DEBUG
		printk("%s:rx %p %d\n", __func__, req, req->actual);
		for(i=0; i<=req->actual; i++)
		{
			printk("%s:buf[%d]=%x\n", __func__, i, req_buf[i]);	
		}
#endif

		if (req->actual == 12)
		{
			if(req_buf[0] == REQUEST_DEV_DEP_MSG_IN)	
			{
				u8 btag = req_buf[1];
				u8 btag_rev = ~req_buf[2];

				if(btag == btag_rev)	
				{
	  				dev->usb_tmc_transfer.bTag = req_buf[1];
					dev->usbtmc_last_read_bTag = req_buf[1];
					//dev->usb_tmc_transfer.bTag++;
					if(!dev->usb_tmc_transfer.bTag)
					{
						//dev->usb_tmc_transfer.bTag++;
					}

					n_characters=req_buf[4]+(req_buf[5]<<8)+(req_buf[6]<<16)+(req_buf[7]<<24);
					dev->term_char_enabled = ((req_buf[8])&0x2)>>1;
					dev->term_char = req_buf[9];
#ifdef TMC_DEBUG
					printk("%s:%d, %d, 0x%x\n", __func__, n_characters, dev->term_char_enabled, dev->term_char);
#endif

					tmc_unlock(&dev->read_excl);
					return n_characters;
				}
				else
				{
					printk("REQUEST_DEV_DEP_MSG_IN btag and btag rev wrong:0x%x, 0x%x\n", btag, btag_rev);	
					r = -EIO;
					tmc_unlock(&dev->read_excl);
					return r;
				}
			}
			else
			{
				printk("not REQUEST_DEV_DEP_MSG_IN, %d\n", req_buf[0]);
				r = -EIO;
				tmc_unlock(&dev->read_excl);
				return r;
			}
		}
		else
		{
			printk("rx %p %d, wrong nm\n", req, req->actual);
			tmc_unlock(&dev->read_excl);
			r = -EIO;
			return r;
		}
	}
}

static ssize_t tmc_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct tmc_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count, xfer, remaining;
	int ret;
	ssize_t  max_transfer_size;
	u8 term_char_enabled = 0;
	u8 term_char= 0;
	u8 *req_buf;

        /*u8 buf_enter[2] ={0};
        copy_from_user(buf_enter,buf,2);

        if(count ==2 && buf_enter[0] == '\r'&& buf_enter[1]=='\n')
            return 2;*/

#ifdef TMC_DEBUG
	printk("tmc_write: write cnt is%d\n", count);
#endif


	if (!_tmc_dev)
		return -ENODEV;

	if (tmc_lock(&dev->write_excl))
		return -EBUSY;

	/*wait state is get ready to write*/
	req = 0;
	ret = wait_event_interruptible(dev->write_wq,
		(req = tmc_req_get(dev, &dev->tx_idle)));
	if (ret<0 || req==0) 
	{
		printk("wait state to get ready to write wrong\n");
		r = ret;
		if(req)
		{
			tmc_req_put(dev, &dev->tx_idle, req);
		}
		tmc_unlock(&dev->write_excl);
		return r;
	}

       	req_buf = (u8 *)req->buf;
	remaining = count;
#ifdef TMC_DEBUG
	printk("remaining %d\n", remaining);
#endif

	while (remaining > 0) 
	{
		max_transfer_size = tmc_read_REQUEST_DEV_DEP_MSG_IN(dev);

		if(max_transfer_size < 0)
		{
			r = -EIO;
			printk("get pc max transfer size wrong\n");
			break;
		}
		else
		{
			term_char_enabled = dev->term_char_enabled;
			term_char = dev->term_char;
#ifdef TMC_DEBUG
			printk("get pc max transfer size %d, %d, %c\n", max_transfer_size, term_char_enabled, term_char);
#endif
		}

		{
			if(remaining > TMC_BULK_BUFFER_SIZE-12-3)
			{
				xfer = TMC_BULK_BUFFER_SIZE-12-3;
			}
			else
			{
				xfer = remaining;	
			}

			if (xfer > max_transfer_size)
			{
				xfer = max_transfer_size;
				req_buf[8] = 0;
			}
			else
			{
				req_buf[8] = 1;
			}

			/*get user's data*/
			req_buf[0]=DEV_DEP_MSG_IN; 
			req_buf[1]=dev->usb_tmc_transfer.bTag; // Transfer ID (bTag)
			req_buf[2]=~(dev->usb_tmc_transfer.bTag); // Inverse of bTag
			req_buf[3]=0; // Reserved
			req_buf[4]=xfer&255; // Transfer size (first byte)
			req_buf[5]=(xfer>>8)&255; // Transfer size (second byte)
			req_buf[6]=(xfer>>16)&255; // Transfer size (third byte)
			req_buf[7]=(xfer>>24)&255; // Transfer size (fourth byte)
			//req_buf[8] is set above...
			req_buf[9]=0; // Reserved
			req_buf[10]=0; // Reserved
			req_buf[11]=0; // Reserved

			if (copy_from_user(req->buf+12, buf, xfer)) 
			{
				printk("copy data from user error\n");
				r = -EFAULT;
				break;
			}

			/*Add zero bytes to achieve 4-byte alignment*/
			req->length = xfer+12;
			if(xfer%4) 
			{
				int n;

				req->length += 4-xfer%4;
				count += 4-xfer%4;
				for(n=12+xfer; n<req->length; n++)
				{
					req_buf[n]=0;
				}
			}
			
			/*send the pdu*/
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) 
			{
				printk("tmc_write: xfer error %d\n", ret);
				r = -EIO;
				break;
			}

			dev->usbtmc_last_write_bTag = dev->usb_tmc_transfer.bTag;
			//dev->usb_tmc_transfer.bTag++;
			if(!dev->usb_tmc_transfer.bTag)
			{
				//dev->usb_tmc_transfer.bTag++;
			}

			buf += xfer;
			remaining -= xfer;
		}
	}

	if(req)
	{
		tmc_req_put(dev, &dev->tx_idle, req);
	}

	tmc_unlock(&dev->write_excl);

	if(r < 0)
	{
		dev->usb_tmc_transfer.nbytes_txd= 0;
#ifdef TMC_DEBUG
		printk("tmc_write success returning  %d\n", r);
#endif
		return r;
	}
	else
	{
		dev->usb_tmc_transfer.nbytes_txd= count-remaining;
#ifdef TMC_DEBUG
		printk("tmc_write wrong returning %d\n", count - remaining);
#endif
		return count - remaining;
	}
}

#define TMC_IOCTL_MAGIC 'T'
#define TMC_IOC_SEND_INT_REQ	_IO(TMC_IOCTL_MAGIC, 1)
//#define TMC_ERR_PAYLOAD_STUCK       _IOW(TMC_IOCTL_MAGIC, 0, unsigned)
//#define TMC_ATS_ENABLE              _IOR(TMC_IOCTL_MAGIC, 1, unsigned)

static long tmc_ioctl(struct file *fp,unsigned int cmd,unsigned long arg)
{
	int ret = 0;
	struct tmc_dev *dev = fp->private_data;

	if (!_tmc_dev)
		return -ENODEV;

	if (tmc_lock(&dev->ioctl_excl))
		return -EBUSY;

	switch(cmd)
	{
		case TMC_IOC_SEND_INT_REQ:	
		{
			struct usb_request *req;
			u8 *req_buf;
			uint32_t bytes;
			u8 *user_buf;

			req = dev->int_in_req;
       			req_buf = (u8 *)req->buf;

			bytes = *((uint32_t *)(arg));
			printk("tmc_ioctl: bytes:%u\n", bytes);
			if(bytes <= 0)
			{
				tmc_unlock(&dev->ioctl_excl);
			       	return -EINVAL;
			}
			if(bytes > 512) 
			{
				tmc_unlock(&dev->ioctl_excl);
				return -EINVAL;
			}

			user_buf = (u8 *)arg;
			if (copy_from_user(req_buf, user_buf+sizeof(uint32_t), bytes))
			{
				tmc_unlock(&dev->ioctl_excl);
				return -EFAULT;
			}
			req->length = bytes;

			ret = usb_ep_queue(dev->ep_int_in, req, GFP_ATOMIC);
			if (ret < 0) 
			{
				tmc_unlock(&dev->ioctl_excl);
				printk("tmc_ioctl: xfer error %d\n", ret);
				ret = -EIO;
				return ret;
			}
		}
		default:
		{
			ret = -EINVAL;
		}
	}
	tmc_unlock(&dev->ioctl_excl);

	return ret;
}

static int tmc_open(struct inode *ip, struct file *fp)
{
#ifdef TMC_DEBUG
	printk(KERN_INFO "[USB] tmc_open: %s(parent:%s): tgid=%d\n",
			current->comm, current->parent->comm, current->tgid);
#endif
	if (!_tmc_dev)
		return -ENODEV;

#if 0
	if (tmc_lock(&_tmc_dev->open_excl))
		return -EBUSY;
#endif

	fp->private_data = _tmc_dev;

	return 0;
}

static int tmc_release(struct inode *ip, struct file *fp)
{
#ifdef TMC_DEBUG
	printk(KERN_INFO "[USB] tmc_release: %s(parent:%s): tgid=%d\n",
			current->comm, current->parent->comm, current->tgid);
#endif
#if 0
	tmc_unlock(&_tmc_dev->open_excl);
#endif
	return 0;
}

#if 0
static DECLARE_WAIT_QUEUE_HEAD(button_waitq); 
//wake_up_interruptible(&button_waitq);
static unsigned int tmc_poll(struct file *file, poll_table *wait)  
{  
    unsigned int mask = 0;  
  
    poll_wait(file, &button_waitq, wait);  
  
    if(1)  
    {  
        mask |= POLLIN | POLLRDNORM;
    }  
  
    return mask;    
}  
#endif

static const struct file_operations tmc_fops = 
{
	.owner = THIS_MODULE,
	.read = tmc_read,
	.write = tmc_write,
	.open = tmc_open,
        .unlocked_ioctl	= tmc_ioctl,
	.release = tmc_release,
	//.poll    = tmc_poll,  
};

static struct miscdevice tmc_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = tmc_shortname,
	.fops = &tmc_fops,
};

static int
tmc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct tmc_dev	*dev = func_to_tmc(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
#ifdef TMC_DEBUG
	printk("tmc_function_bind dev: %p\n", dev);
#endif
	
	id = usb_interface_id(c, f);
	if (id < 0)
	{
		printk("%s:id<0\n", __func__);
		return id;
	}

	tmc_interface_desc.bInterfaceNumber = id;
	
	ret = tmc_create_bulk_endpoints(dev, &tmc_fullspeed_in_desc,
			&tmc_fullspeed_out_desc, &tmc_fullspeed_int_in_desc);
	if (ret)
		return ret;

	
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		tmc_highspeed_in_desc.bEndpointAddress =
			tmc_fullspeed_in_desc.bEndpointAddress;
		tmc_highspeed_out_desc.bEndpointAddress =
			tmc_fullspeed_out_desc.bEndpointAddress;
		tmc_highspeed_int_in_desc.bEndpointAddress =
			tmc_fullspeed_int_in_desc.bEndpointAddress;
	}
	printk("%s speed %s: IN/%s, OUT/%s, INT%s/\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name, dev->ep_int_in->name);

	ret = usb_assign_descriptors(f, fs_tmc_descs, hs_tmc_descs, NULL);
	if (ret)
		return ret;

#ifdef TMC_DEBUG
	printk("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
#endif
	return 0;
}

static void
tmc_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct tmc_dev	*dev = func_to_tmc(f);
	struct usb_request *req;

	atomic_set(&dev->online, 0);
	//wake_up(&dev->read_wq);

	tmc_request_free(dev->rx_req, dev->ep_out);
	while ((req = tmc_req_get(dev, &dev->tx_idle)))
		tmc_request_free(req, dev->ep_in);
}

static int tmc_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct tmc_dev	*dev = func_to_tmc(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

#ifdef TMC_DEBUG
	printk("tmc_function_set_alt intf: %d alt: %d\n", intf, alt);
#endif

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
			dev->ep_in->name, ret);
		return ret;
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		dev->ep_out->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
			dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
				dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_int_in);
	if (ret) {
		dev->ep_int_in->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
			dev->ep_int_in->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_int_in);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
				dev->ep_int_in->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}

	atomic_set(&dev->online, 1);
	wake_up(&dev->read_wq);

	return 0;
}

static void tmc_function_disable(struct usb_function *f)
{
	struct tmc_dev	*dev = func_to_tmc(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	atomic_set(&dev->online, 0);
	usb_ep_disable(dev->ep_in);
#ifdef TMC_DEBUG
	printk("tmc_function_disable 2:%p\n", dev->ep_out);
#endif
	usb_ep_disable(dev->ep_out);
#ifdef TMC_DEBUG
	printk("tmc_function_disable 3\n");
#endif
	usb_ep_disable(dev->ep_int_in);
	wake_up(&dev->read_wq);

#ifdef TMC_DEBUG
	printk("%s disabled\n", dev->function.name);
#endif
}

static int tmc_function_setup(struct usb_function *f, const struct usb_ctrlrequest *c)
{
    int value = -EOPNOTSUPP;
    u16 wIndex = le16_to_cpu(c->wIndex);
    u16 wValue = le16_to_cpu(c->wValue);
    u16 wLength = le16_to_cpu(c->wLength);
    struct tmc_dev	*dev = func_to_tmc(f);
    struct usb_composite_dev *cdev = dev->cdev;
    struct usb_request *req = cdev->req;

#ifdef TMC_DEBUG
    printk("%s:%d, %d\n", __func__, c->bRequestType & USB_TYPE_MASK, c->bRequest);
#endif
    printk("%s:%d, %d\n", __func__, c->bRequestType & USB_TYPE_MASK, c->bRequest);

    switch (c->bRequestType & USB_TYPE_MASK) 
    {
        case USB_TYPE_CLASS:
            switch (c->bRequest) 
            {
                case USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT:
                    /* check if the active transfer has the requested bTag value */
#ifdef TMC_DEBUG
                    printk("%s:USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_IN)
                    {
                        if(dev->usb_tmc_transfer.bTag == wValue) 
                        {
                            dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;

                            usb_ep_set_halt(dev->ep_out);
                            usb_ep_fifo_flush(dev->ep_out);
                        }
                        else 
                        {
                            dev->usb_tmc_status = USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
                        }

                        dev->buf[0] = dev->usb_tmc_status;
                        dev->buf[1] = dev->usb_tmc_transfer.bTag;
                        value = 2;
                        req->zero = 0;
                        req->length = value;
                        memcpy(req->buf, dev->buf, value);
                        if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
                        {
                            printk(KERN_ERR "ep0 in queue failed\n");
                        }
                    }

                    break;
                case USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS:
#ifdef TMC_DEBUG
                    printk("%s:USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_IN)
                    {
                        /* send number of transmitted bytes */ 
                        dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
                        dev->buf[0] = dev->usb_tmc_status;
                        dev->buf[1] = 0;
                        dev->buf[2] = 0;
                        dev->buf[3] = 0;
                        dev->buf[4] = (dev->usb_tmc_transfer.nbytes_rxd & 0x0000FF);
                        dev->buf[5] = (dev->usb_tmc_transfer.nbytes_rxd & 0x00FF00)>>8;
                        dev->buf[6] = (dev->usb_tmc_transfer.nbytes_rxd & 0xFF0000)>>16;
                        dev->buf[7] = dev->usb_tmc_transfer.nbytes_rxd >>24;

                        value = 8;
                        req->zero = 0;
                        req->length = value;
                        memcpy(req->buf, dev->buf, value);
                        if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
                        {
                            printk(KERN_ERR "ep0 in queue failed\n");
                        }
                    }
                    break;
                case USBTMC_REQUEST_INITIATE_ABORT_BULK_IN:
#ifdef TMC_DEBUG
                    printk("%s:USBTMC_REQUEST_INITIATE_ABORT_BULK_IN\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_IN)
                    {
                        /* check if the active transfer has the requested bTag value */
                        if(dev->usb_tmc_transfer.bTag == wValue) 
                        {
                            dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
                            //usb_ep_set_halt(dev->ep_out);
                            usb_ep_set_halt(dev->ep_in);
                            usb_ep_fifo_flush(dev->ep_in);
                        }
                        else 
                        {
                            dev->usb_tmc_status = USBTMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
                        }

                        dev->buf[0] = dev->usb_tmc_status;
                        dev->buf[1] = wValue;
                        value = 2;
                        req->zero = 0;
                        req->length = value;
                        memcpy(req->buf, dev->buf, value);
                        if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
                        {
                            printk(KERN_ERR "ep0 in queue failed\n");
                        }
                    }
                    break;
                case USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS:
#ifdef TMC_DEBUG
                    printk("%s:USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_IN)
                    {
                        /* send number of transmitted bytes */ 
                        dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
                        dev->buf[0] = dev->usb_tmc_status;
                        dev->buf[1] = 0;
                        dev->buf[2] = 0;
                        dev->buf[3] = 0;
                        dev->buf[4] = (dev->usb_tmc_transfer.nbytes_txd & 0x0000FF);
                        dev->buf[5] = (dev->usb_tmc_transfer.nbytes_txd & 0x00FF00)>>8;
                        dev->buf[6] = (dev->usb_tmc_transfer.nbytes_txd & 0xFF0000)>>16;
                        dev->buf[7] = dev->usb_tmc_transfer.nbytes_txd >>24;

                        value = 8;
                        req->zero = 0;
                        req->length = value;
                        memcpy(req->buf, dev->buf, value);
                        if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
                        {
                            printk(KERN_ERR "ep0 in queue failed\n");
                        }
                    }
                    break;
                case USBTMC_REQUEST_INITIATE_CLEAR:
#ifdef TMC_DEBUG
                    printk("%s:USBTMC_REQUEST_INITIATE_CLEAR\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_IN)
                    {
                        dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
                        usb_ep_set_halt(dev->ep_in);
                        usb_ep_set_halt(dev->ep_out);
                        //usb_ep_fifo_flush(dev->ep_in);
                        //usb_ep_fifo_flush(dev->ep_out);

                        dev->buf[0] = dev->usb_tmc_status;
                        value = 1;
                        req->zero = 0;
                        req->length = value;
                        memcpy(req->buf, dev->buf, value);
                        if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
                        {
                            printk(KERN_ERR "ep0 in queue failed\n");
                        }
                    }
                    break;
                case USBTMC_REQUEST_CHECK_CLEAR_STATUS:
#ifdef TMC_DEBUG
                    printk("%s:USBTMC_REQUEST_CHECK_CLEAR_STATUS\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_IN)
                    {
                        dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
                        dev->buf[0] = dev->usb_tmc_status;
                        dev->buf[1] = wValue;
                        value = 2;
                        req->zero = 0;
                        req->length = value;
                        memcpy(req->buf, dev->buf, value);
                        if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
                        {
                            printk(KERN_ERR "ep0 in queue failed\n");
                        }
                    }
                    break;
                case USBTMC_REQUEST_GET_CAPABILITIES:
#ifdef TMC_DEBUG
                    printk("%s:USBTMC_REQUEST_GET_CAPABILITIES\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_IN)
                    {
                        dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
                        value = sizeof(USB_TMC_Capabilities);
                        req->zero = 0;
                        req->length = value;
                        memcpy(req->buf, (char *)&USB_TMC_CAPABILITIES, value);
                        if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
                        {
                            printk(KERN_ERR "ep0 in queue failed\n");
                        }
                    }
                    break;
                case USBTMC_REQUEST_INDICATOR_PULSE:
                    /* optional command, not implemented */
                    value = 0;
                    break;
                default:
                    value = 0;
                    break;
            }
            break;
        case USB_TYPE_STANDARD:
            switch (c->bRequest) 
            {
                case USB_REQ_CLEAR_FEATURE:
#ifdef TMC_DEBUG
                    printk("%s:USB_REQ_CLEAR_FEATURE\n", __func__);
#endif
                    //if (c->bRequestType == USB_DIR_OUT)
                    {
                        if(wValue == USB_ENDPOINT_HALT)	
                        {
                            if(wIndex == 0)
                            {
                                usb_ep_clear_halt(dev->ep_out);
                            }
                            else
                            {
                                usb_ep_clear_halt(dev->ep_in);
                            }
                        }
                    }
                    break;
            }
        default:
            break;
    }

    return value;
}

static int tmc_bind_config(struct usb_configuration *c)
{
	struct tmc_dev *dev = _tmc_dev;

#ifdef TMC_DEBUG
	printk(KERN_INFO "tmc_bind_config\n");
#endif

	dev->cdev = c->cdev;
}

static inline struct f_tmc_opts
*to_f_tmc_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_tmc_opts,
			    func_inst.group);
}

CONFIGFS_ATTR_STRUCT(f_tmc_opts);
CONFIGFS_ATTR_OPS(f_tmc_opts);

static void tmc_attr_release(struct config_item *item)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations tmc_item_ops = {
	.release	= tmc_attr_release,
	.show_attribute	= f_tmc_opts_attr_show,
	.store_attribute = f_tmc_opts_attr_store,
};

static ssize_t f_tmc_opts_pnp_string_show(struct f_tmc_opts *opts,
					      char *page)
{
	int result;

	mutex_lock(&opts->lock);
	result = strlcpy(page, opts->pnp_string + 2, PNP_STRING_LEN - 2);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_opts_pnp_string_store(struct f_tmc_opts *opts,
					       const char *page, size_t len)
{
	int result, l;

	mutex_lock(&opts->lock);
	result = strlcpy(opts->pnp_string + 2, page, PNP_STRING_LEN - 2);
	l = strlen(opts->pnp_string + 2) + 2;
	opts->pnp_string[0] = (l >> 8) & 0xFF;
	opts->pnp_string[1] = l & 0xFF;
	mutex_unlock(&opts->lock);

	return result;
}

static struct f_tmc_opts_attribute f_tmc_opts_pnp_string =
	__CONFIGFS_ATTR(pnp_string, S_IRUGO | S_IWUSR,
			f_tmc_opts_pnp_string_show,
			f_tmc_opts_pnp_string_store);

static ssize_t f_tmc_opts_q_len_show(struct f_tmc_opts *opts,
					 char *page)
{
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d\n", opts->q_len);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_opts_q_len_store(struct f_tmc_opts *opts,
					  const char *page, size_t len)
{
	int ret;
	u16 num;

	mutex_lock(&opts->lock);
	if (opts->refcnt) {
		ret = -EBUSY;
		goto end;
	}

	ret = kstrtou16(page, 0, &num);
	if (ret)
		goto end;

	opts->q_len = (unsigned)num;
	ret = len;
end:
	mutex_unlock(&opts->lock);
	return ret;
}

static struct f_tmc_opts_attribute f_tmc_opts_q_len =
	__CONFIGFS_ATTR(q_len, S_IRUGO | S_IWUSR, f_tmc_opts_q_len_show,
			f_tmc_opts_q_len_store);

static struct configfs_attribute *tmc_attrs[] = {
	&f_tmc_opts_pnp_string.attr,
	&f_tmc_opts_q_len.attr,
	NULL,
};

static struct config_item_type tmc_func_type = {
	.ct_item_ops	= &tmc_item_ops,
	.ct_attrs	= tmc_attrs,
	.ct_owner	= THIS_MODULE,
};

static void gtmc_free_inst(struct usb_function_instance *f)
{
	struct f_tmc_opts *opts;

	opts = container_of(f, struct f_tmc_opts, func_inst);

	kfree(opts);
}

static struct usb_function_instance *gtmc_alloc_inst(void)
{
	struct f_tmc_opts *opts;
	struct usb_function_instance *ret;
	int status = 0;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	mutex_init(&opts->lock);
	opts->func_inst.free_func_inst = gtmc_free_inst;
	ret = &opts->func_inst;


	config_group_init_type_name(&opts->func_inst.group, "",
				    &tmc_func_type);

	return ret;
}

static void gtmc_free(struct usb_function *f)
{
	struct tmc_dev *dev = func_to_tmc(f);
	struct f_tmc_opts *opts;

	opts = container_of(f->fi, struct f_tmc_opts, func_inst);

	misc_deregister(&tmc_device);
	kfree(dev);
	_tmc_dev = NULL;

	mutex_lock(&opts->lock);
	--opts->refcnt;
	mutex_unlock(&opts->lock);
}

static void init_usb_tmc(void)
{
	struct tmc_dev *dev = _tmc_dev;

	dev->usb_tmc_transfer.bTag = 0;
	dev->usb_tmc_transfer.nbytes_rxd = 0;
	dev->usb_tmc_transfer.nbytes_txd = 0;
	
	dev->usb_tmc_status = USBTMC_STATUS_SUCCESS;
}

static struct usb_function *gtmc_alloc_func(struct usb_function_instance *fi)
{
    int ret = 0;
	struct tmc_dev	*dev;
	struct f_tmc_opts	*opts;

	opts = container_of(fi, struct f_tmc_opts, func_inst);

	mutex_lock(&opts->lock);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		mutex_unlock(&opts->lock);
		return ERR_PTR(-ENOMEM);
	}

	++opts->refcnt;

	mutex_unlock(&opts->lock);

	dev->function.name = "tmc";
	dev->function.bind = tmc_function_bind;
	dev->function.setup   = tmc_function_setup;
	dev->function.unbind = tmc_function_unbind;
	dev->function.set_alt = tmc_function_set_alt;
	dev->function.disable = tmc_function_disable;
	dev->function.free_func = gtmc_free;

	spin_lock_init(&dev->lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);
	init_waitqueue_head(&dev->int_in_wq);

	//atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);
	atomic_set(&dev->ioctl_excl, 0);

	INIT_LIST_HEAD(&dev->tx_idle);

	_tmc_dev = dev;

	ret = misc_register(&tmc_device);
	if (ret)
		goto err;

	init_usb_tmc();

	return &dev->function;
err:
	kfree(dev);
	printk(KERN_ERR "tmc gadget driver failed to initialize\n");
	return NULL;
}

DECLARE_USB_FUNCTION_INIT(gtmc, gtmc_alloc_inst, gtmc_alloc_func);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zgt_lz");

