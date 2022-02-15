/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/usb_bridge.h>

#define USB_QTI_VENDOR_ID 0x05c6

struct ub_func_inst_wq {
	struct work_struct work;
	struct ub_func *func_data[MAX_CONFIG_INTERFACES];
};

struct ub_debug {
	uint8_t intf;
	uint8_t state;
	unsigned long ub_to_host;
	unsigned long host_to_ub;
	unsigned long ub_to_device;
	unsigned long device_to_ub;
	unsigned long ub_to_host_err;
	unsigned long host_to_ub_err;
	unsigned long ub_to_device_err;
	unsigned long device_to_ub_err;
};

static struct ub_intf_list ub_intf_data;
static struct ub_func_list *ub_func_data;
static struct workqueue_struct *ub_wq;
static struct ub_func_inst_wq ub_func_act;

void usb_bridge_device_in_compl(struct urb *urb)
{
	struct ub_req *ub_req = (struct ub_req *)urb->context;
	int ret;

	if (!urb->status)
		ub_req->device_to_ub++;
	else
		ub_req->device_to_ub_err++;

	if (!urb->status && (ub_req->state != UB_REQ_DO_NOT_SUBMIT)) {
		if (!ub_req)
			return;
		if (!ub_req->req)
			return;

		ub_req->req->length = urb->actual_length;
		ub_req->req->zero = 0;

		if (ub_req->ub_f_ep) {
			ret = usb_ep_queue(ub_req->ub_f_ep, ub_req->req,
					   GFP_ATOMIC);
			if (ret < 0)
				ub_req->state = UB_REQ_FREE;
			else
				ub_req->state = UB_REQ_QUEUE_SUBMIT;
		}
	} else if (urb->status == -EPERM) {
		kfree(urb->transfer_buffer);
		usb_free_urb(urb);
	}
}

void usb_bridge_device_out_compl(struct urb *urb)
{
	struct ub_req *ub_req = (struct ub_req *)urb->context;
	int ret;

	if (!urb->status)
		ub_req->ub_to_device++;
	else
		ub_req->ub_to_device_err++;

	if (!urb->status && (ub_req->state != UB_REQ_DO_NOT_SUBMIT)) {
		/* Data was successfully sent, notify host */
		ret = usb_ep_queue(ub_req->ub_f_ep, ub_req->req, GFP_ATOMIC);
		if (ret < 0)
			ub_req->state = UB_REQ_FREE;
		else
			ub_req->state = UB_REQ_QUEUE_SUBMIT;
	} else if (urb->status == -EPERM) {
		kfree(urb->transfer_buffer);
		usb_free_urb(urb);
	}
}

struct ub_iface *usb_bridge_interface_get(uint8_t func_id)
{
	unsigned long flags = 0;
	struct ub_iface *ub_i_data = NULL;

	/* Find interface with same interface id */
	spin_lock_irqsave(&ub_intf_data.intf_lock, flags);
	list_for_each_entry(ub_i_data, &ub_intf_data.intf_list, intf_hook) {
		if (func_id == ub_i_data->intf_id) {
			spin_unlock_irqrestore(&ub_intf_data.intf_lock,
					       flags);
			return ub_i_data;
		}
	}
	spin_unlock_irqrestore(&ub_intf_data.intf_lock, flags);

	return NULL;
}

static bool usb_bridge_check_vid_pid(struct ub_iface *i, struct ub_func *f)
{
	struct usb_device *udev = i->udev;

	if ((udev->descriptor.idVendor == f->vid) &&
	    (udev->descriptor.idProduct == f->pid))
		return true;

	return false;
}

static void usb_bridge_func_activate(struct work_struct *data)
{
	int i = 0, ret = 0;
	struct usb_gadget *g = NULL;
	struct usb_composite_dev *cdev = NULL;
	struct ub_func_inst_wq *funcs = (struct ub_func_inst_wq *)data;

	for (i = 0; i < MAX_CONFIG_INTERFACES; i++) {
		if (funcs->func_data[i] != NULL) {
			cdev = funcs->func_data[i]->function.config->cdev;
			g = cdev->gadget;
			if (!cdev->deactivations) {
				dev_dbg(&g->dev, "Gadget already active\n");
				return;
			}
			if (pm_runtime_status_suspended(g->dev.parent))
				pm_runtime_irq_safe(cdev->gadget->dev.parent);

			ret = usb_function_activate(
				&funcs->func_data[i]->function);
			if (!ret)
				funcs->func_data[i]->func_state = UB_FUNC_ACT;
			funcs->func_data[i] = NULL;
		}
	}
}

static int usb_bridge_func_add(struct ub_iface *ub_i_data,
			       uint8_t interface_num)
{
	struct ub_func *func_data = NULL;
	struct device *dev = NULL;
	unsigned long flags;
	uint8_t func_add = false;
	int ret = -ENODEV;
	int i = 0;

	/* Function is registered, check if function instance exists */
	spin_lock_irqsave(&ub_func_data->func_lock, flags);
	list_for_each_entry(func_data, &ub_func_data->func_list, func_hook) {
		if (func_data->iface_name != interface_num)
			continue;

		if ((func_data->function.bind == NULL) ||
		    (func_data->iface_data == NULL))
			break;

		if (usb_bridge_check_vid_pid(ub_i_data, func_data)) {
			func_add = true;
			func_data->iface_data = ub_i_data;
			break;
		}

		dev_info(&ub_i_data->interface->dev,
		"Intf %d VID/PID (%04x:%04x) does not match bridged VID/PID (%04x:%04x)",
		interface_num,
		ub_i_data->udev->descriptor.idVendor,
		ub_i_data->udev->descriptor.idProduct,
		func_data->vid, func_data->pid);
		break;
	}
	spin_unlock_irqrestore(&ub_func_data->func_lock, flags);

	if (func_add) {
		dev = &func_data->config->cdev->gadget->dev;
		/* Claim interface */
		usb_set_intfdata(ub_i_data->interface, ub_i_data);
		if (ub_i_data->interface->cur_altsetting->string != NULL)
			dev_info(dev, "Claimed interface %s of 0x%04x:0x%04x\n",
				 ub_i_data->interface->cur_altsetting->string,
				 ub_i_data->udev->descriptor.idVendor,
				 ub_i_data->udev->descriptor.idProduct);
		else
			dev_info(dev, "Claimed interface %d of 0x%04x:0x%04x\n",
				 interface_num,
				 ub_i_data->udev->descriptor.idVendor,
				 ub_i_data->udev->descriptor.idProduct);

		for (i = 0; i < MAX_CONFIG_INTERFACES; i++) {
			if (ub_func_act.func_data[i] == NULL) {
				ub_func_act.func_data[i] = func_data;
				break;
			}
		}
		if (i == MAX_CONFIG_INTERFACES)
			return -ENOBUFS;

		queue_work(ub_wq, &ub_func_act.work);

		ret = 0;
	}

	return ret;
}

static int usb_bridge_probe(struct usb_interface *interface,
			    const struct usb_device_id *dev_id)
{
	struct ub_iface *ub_i_data = NULL;
	struct usb_device *udev = NULL;
	struct usb_endpoint_descriptor *epd = NULL;
	struct usb_host_interface *cur_altsetting = NULL;
	int ret = -ENODEV;
	int num_ep = 0, i = 0;
	unsigned long flags;
	struct ub_ep_count *count = NULL;

	if (interface->cur_altsetting) {
		cur_altsetting = interface->cur_altsetting;
		ub_i_data = (struct ub_iface *)
			     kzalloc(sizeof(struct ub_iface), GFP_ATOMIC);
		if (!ub_i_data) {
			dev_dbg(&interface->dev,
				"Unable to alloc interface data\n");
			return -ENOMEM;
		}

		udev = usb_get_dev(interface_to_usbdev(interface));
		ub_i_data->udev = udev;
		ub_i_data->interface = interface;

		ub_i_data->intf_id = cur_altsetting->desc.bInterfaceNumber;
		count = &ub_i_data->ep_count;

		/* Get number of endpoints */
		num_ep = cur_altsetting->desc.bNumEndpoints;

		for (i = 0; i < num_ep; i++) {
			epd = &cur_altsetting->endpoint[i].desc;
			if (usb_endpoint_is_bulk_in(epd) &&
			    (count->bulk_in < USB_MAXCONFIG))
				ub_i_data->b_in_ep[count->bulk_in++] = *epd;
			else if (usb_endpoint_is_bulk_out(epd) &&
				 (count->bulk_out < USB_MAXCONFIG))
				ub_i_data->b_out_ep[count->bulk_out++] = *epd;
		}

		/* Add interface to list */
		spin_lock_irqsave(&ub_intf_data.intf_lock, flags);
		list_add(&(ub_i_data->intf_hook), &(ub_intf_data.intf_list));
		spin_unlock_irqrestore(&ub_intf_data.intf_lock, flags);

		ret = usb_bridge_func_add(ub_i_data,
			cur_altsetting->desc.bInterfaceNumber);
	}
	return ret;
}

static void usb_bridge_disconnect(struct usb_interface *interface)
{
	uint8_t func_bridged = false;
	unsigned long flags;
	struct ub_iface *ub_i_data = NULL;
	struct ub_func *func_data = NULL;
	struct usb_host_interface *cur_altsetting = NULL;
	int ret = 0;

	spin_lock_irqsave(&ub_intf_data.intf_lock, flags);
	list_for_each_entry(ub_i_data, &ub_intf_data.intf_list, intf_hook) {
		cur_altsetting = ub_i_data->interface->cur_altsetting;
		if (cur_altsetting->desc.bInterfaceNumber ==
		    interface->cur_altsetting->desc.bInterfaceNumber) {
			list_del(&ub_i_data->intf_hook);
			goto remove;
		}
	}

remove:
	spin_unlock_irqrestore(&ub_intf_data.intf_lock, flags);

	/* Disconnect the function that might be bridged to this interface */
	spin_lock_irqsave(&ub_func_data->func_lock, flags);
	list_for_each_entry(func_data, &ub_func_data->func_list, func_hook) {
		if (func_data->iface_name ==
			interface->cur_altsetting->desc.bInterfaceNumber) {
			func_bridged = true;
			goto remove_func;
		}
	}

remove_func:
	spin_unlock_irqrestore(&ub_func_data->func_lock, flags);
	if (func_bridged) {
		if (func_data->bind) {
			ret = usb_function_deactivate(&func_data->function);
			dev_dbg(&interface->dev,
				"Function deactivate status %d\n",
				ret);
			if (!ret)
				func_data->func_state = UB_FUNC_DEACT;

			func_data->function.disable(&func_data->function);
		}
	}

	kfree(ub_i_data);

	/* Release Interface */
	usb_set_intfdata(interface, NULL);
}

static const struct usb_device_id ub_device_table[] = {
	{ USB_DEVICE(USB_QTI_VENDOR_ID, 0x900E) },
	{ USB_DEVICE(USB_QTI_VENDOR_ID, 0x901D) },
	{ USB_DEVICE(USB_QTI_VENDOR_ID, 0x90EF) },
	{ USB_DEVICE(USB_QTI_VENDOR_ID, 0x90F0) },
#if 1 // add for quectel usb config
	{ USB_DEVICE(0x2c7c, 0x0415) },
#endif
	{ } /* Terminating entry */
};

static struct usb_driver usb_bridge_intf_driver = {
	.name = "usb_bridge",
	.probe = usb_bridge_probe,
	.disconnect = usb_bridge_disconnect,
	.id_table = ub_device_table,
	.supports_autosuspend = 0,
};

int usb_bridge_interface_claim(struct ub_iface *ub_i_data)
{
	int ret = 0;
	void *priv = NULL;

	ret = usb_driver_claim_interface(&usb_bridge_intf_driver,
					 ub_i_data->interface, ub_i_data);
	if (ret) {
		/* Do we already have the interface? */
		priv = usb_get_intfdata(ub_i_data->interface);
		if (priv == (void *)ub_i_data)
			ret = 0;
	}
	dev_dbg(&ub_i_data->interface->dev, "Interface claim status %d\n",
		ret);
	return ret;
}

void usb_bridge_interface_release(struct usb_interface *interface)
{
	usb_driver_release_interface(&usb_bridge_intf_driver, interface);
}

static int usb_bridge_status(struct seq_file *s, void *unused)
{
	struct ub_func *func = NULL;
	struct ub_debug dbg[USB_MAXCONFIG] = {0};
	int fcount = 0;
	int nume = 0;
	int num_funcs = 0;
	unsigned long flags;

	/* Read counters */
	spin_lock_irqsave(&ub_func_data->func_lock, flags);
	list_for_each_entry(func, &ub_func_data->func_list, func_hook) {
		dbg[fcount].intf = func->iface_name;
		dbg[fcount].state = func->enabled;
		if (dbg[fcount].state) {
			for (nume = 0; nume < func->num_ep; nume++) {
				dbg[fcount].host_to_ub +=
					func->ub_reqs[nume]->host_to_ub;
				dbg[fcount].ub_to_device +=
					func->ub_reqs[nume]->ub_to_device;
				dbg[fcount].device_to_ub +=
					func->ub_reqs[nume]->device_to_ub;
				dbg[fcount].ub_to_host +=
					func->ub_reqs[nume]->ub_to_host;
				dbg[fcount].host_to_ub_err +=
					func->ub_reqs[nume]->host_to_ub_err;
				dbg[fcount].ub_to_device_err +=
					func->ub_reqs[nume]->ub_to_device_err;
				dbg[fcount].device_to_ub_err +=
					func->ub_reqs[nume]->device_to_ub_err;
				dbg[fcount].ub_to_host_err +=
					func->ub_reqs[nume]->ub_to_host_err;
			}
		}
		fcount++;
	}
	spin_unlock_irqrestore(&ub_func_data->func_lock, flags);

	num_funcs = fcount;

	/* Print header */
	seq_printf(s, "%s\t%s\t%s\t%s\t%s\t%s\n",
			"interface",
			"status",
			"host->bridge",
			"bridge->device",
			"device->bridge",
			"bridge->host");

	/* Print counters */
	for (fcount = 0; fcount < num_funcs; fcount++) {
		seq_printf(s, "%ld\t\t\t%s\t",
				dbg[fcount].intf,
				dbg[fcount].state ? "Bridged" : "Not Bridged");
		seq_printf(s, "%ld\t\t%ld\t\t",
				dbg[fcount].host_to_ub,
				dbg[fcount].ub_to_device);
		seq_printf(s, "%ld\t\t%ld",
				dbg[fcount].device_to_ub,
				dbg[fcount].ub_to_host);
	}

	/* Print error counters */
	seq_printf(s, "\n\nErrors:\n");
	for (fcount = 0; fcount < num_funcs; fcount++) {
		seq_printf(s, "%s\t%s\t%s\t%s\n",
				"host->bridge",
				"bridge->device",
				"device->bridge",
				"bridge->host");
		seq_printf(s, "%ld\t\t%ld\t\t",
				dbg[fcount].host_to_ub_err,
				dbg[fcount].ub_to_device_err);
		seq_printf(s, "%ld\t\t%ld",
				dbg[fcount].device_to_ub_err,
				dbg[fcount].ub_to_host_err);
	}
	seq_printf(s, "\n");

	return 0;
}

static int usb_bridge_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, usb_bridge_status, inode->i_private);
}

const struct file_operations ub_file_ops = {
	.open = usb_bridge_status_open,
	.read =  seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init usb_bridge_init(void)
{
	int ret = 0;
	struct dentry *ub_dentry = NULL;
	struct dentry *ub_dentry_root = NULL;

	/* Initialize interface list */
	INIT_LIST_HEAD(&ub_intf_data.intf_list);
	spin_lock_init(&ub_intf_data.intf_lock);

	/* Create workqueue */
	ub_wq = create_workqueue("ub_intf_wq");
	INIT_WORK(&ub_func_act.work, usb_bridge_func_activate);

	ub_func_data = usb_bridge_func_init(&ub_intf_data);
	if (ub_func_data == NULL)
		return -EEXIST;

	/* Register Interface Driver (Host) */
	ret = usb_register_driver(&usb_bridge_intf_driver, THIS_MODULE,
				  KBUILD_MODNAME);
	if (ret) {
		pr_err("USB bridge init fail\n");
		return ret;
	}

	ub_intf_data.func_register = true;

	/* Setup debug interface */
	ub_dentry_root = debugfs_create_dir("usb_bridge", NULL);
	if (ub_dentry_root && !IS_ERR(ub_dentry_root)) {
		ub_dentry = debugfs_create_file("status",
						S_IRUGO | S_IWUSR,
						ub_dentry_root,
						&ub_intf_data,
						&ub_file_ops);
		if (!ub_dentry)
			debugfs_remove_recursive(ub_dentry_root);
	}

	if (!ub_dentry || !ub_dentry_root)
		pr_err("Unable to create debug node\n");

	ub_intf_data.dentry_root = ub_dentry_root;
	return ret;
}

static void __exit usb_bridge_exit(void)
{
	usb_deregister(&usb_bridge_intf_driver);

	if (ub_intf_data.func_register)
		usb_bridge_func_unreg();

	debugfs_remove_recursive(ub_intf_data.dentry_root);
}

module_init(usb_bridge_init);
module_exit(usb_bridge_exit);

MODULE_LICENSE("GPL v2");
