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

#include <linux/usb_bridge.h>
#include <linux/version.h>

struct ub_func_en_wq {
	struct work_struct work;
	struct ub_func *func_data[MAX_CONFIG_INTERFACES];
	spinlock_t wq_lock;
};

struct ub_func_desc_hdr {
	struct usb_descriptor_header **fs;
	struct usb_descriptor_header **hs;
	struct usb_descriptor_header **ss;
};

static struct workqueue_struct *ub_f_wq;
static struct ub_func_en_wq ub_f_setup;
static struct ub_func_list ub_func_data;
static struct ub_intf_list *ub_intf_data;

static void usb_bridge_attr_release(struct config_item *item)
{
	struct config_group *group = NULL;
	struct usb_function_instance *fi = NULL;

	group = container_of(item, struct config_group, cg_item);
	fi = container_of(group, struct usb_function_instance, group);

	usb_put_function_instance(fi);
}

static void usb_bridge_host_in_compl(struct usb_ep *ep, struct usb_request *req)
{
	struct ub_req *ub_req = req->context;
	int ret;

	if (req->status && req->status != -ESHUTDOWN)
		ub_req->ub_to_host_err++;
	else if (!req->status)
		ub_req->ub_to_host++;

	/* Send completion event to Device */
	if (!req->status && (ub_req->state != UB_REQ_DO_NOT_SUBMIT)) {
		ret = usb_submit_urb(ub_req->urb, GFP_ATOMIC);
		if (ret)
			ub_req->state = UB_REQ_FREE;
		else
			ub_req->state = UB_REQ_URB_SUBMIT;
	}
}

static void usb_bridge_host_out_compl(struct usb_ep *ep,
				      struct usb_request *req)
{
	struct ub_req *ub_req = req->context;
	int ret;

	if (req->status && req->status != -ESHUTDOWN)
		ub_req->host_to_ub_err++;
	else if (!req->status)
		ub_req->host_to_ub++;

	/* Send data to Device */
	if (!req->status && (ub_req->state != UB_REQ_DO_NOT_SUBMIT)) {
		ub_req->urb->transfer_buffer_length = req->actual;

		ret = usb_submit_urb(ub_req->urb, GFP_ATOMIC);
		if (ret < 0)
			ub_req->state = UB_REQ_FREE;
		else
			ub_req->state = UB_REQ_URB_SUBMIT;
	}
}

static void usb_bridge_fill_bulk_ep_desc(struct ub_func *func_data, int index,
						   uint8_t dir)
{
	struct usb_endpoint_descriptor *dest;
	struct usb_ss_ep_comp_descriptor *comp_dest;

	/* Fill FS descriptor */
	dest = &func_data->fs_eps[index];
	dest->bLength           = USB_DT_ENDPOINT_SIZE;
	dest->bDescriptorType   = USB_DT_ENDPOINT;
	dest->bEndpointAddress  = dir;
	dest->bmAttributes      = USB_ENDPOINT_XFER_BULK;

	/* Fill HS descriptor */
	dest = &func_data->hs_eps[index];
	dest->bLength           = USB_DT_ENDPOINT_SIZE;
	dest->bDescriptorType   = USB_DT_ENDPOINT;
	dest->bmAttributes      = USB_ENDPOINT_XFER_BULK;
	dest->wMaxPacketSize    = cpu_to_le16(USB_DESC_BUFSIZE);

	/* Fill SS descriptor */
	dest = &func_data->ss_eps[index];
	dest->bLength           = USB_DT_ENDPOINT_SIZE;
	dest->bDescriptorType   = USB_DT_ENDPOINT;
	dest->bmAttributes      = USB_ENDPOINT_XFER_BULK;
	dest->wMaxPacketSize    = cpu_to_le16(2*USB_DESC_BUFSIZE);

	/* Fill SS COMP descriptor */
	comp_dest = &func_data->ss_comp_eps[index];
	comp_dest->bLength              = USB_DT_SS_EP_COMP_SIZE;
	comp_dest->bDescriptorType      = USB_DT_SS_ENDPOINT_COMP;
	comp_dest->bMaxBurst            = 0;
	comp_dest->bmAttributes         = 0;
	comp_dest->wBytesPerInterval    = 0;
}

static struct usb_interface_assoc_descriptor
*usb_bridge_alloc_assoc_desc(struct usb_interface_assoc_descriptor *src)
{
	struct usb_interface_assoc_descriptor *desc = NULL;
	size_t len = sizeof(struct usb_interface_assoc_descriptor);

	desc = kzalloc(len, GFP_ATOMIC);
	if (!desc)
		return NULL;

	desc->bLength = src->bLength;
	desc->bDescriptorType = src->bDescriptorType;
	desc->bInterfaceCount = src->bInterfaceCount;
	desc->bFunctionClass = src->bFunctionClass;
	desc->bFunctionSubClass = src->bFunctionSubClass;
	desc->bFunctionProtocol = src->bFunctionProtocol;
	desc->iFunction = src->iFunction;

	return desc;
}

static struct usb_interface_descriptor
*usb_bridge_alloc_iface_desc(struct usb_interface_descriptor *src)
{
	struct usb_interface_descriptor *desc = NULL;
	size_t len = sizeof(struct usb_interface_descriptor);

	desc = kzalloc(len, GFP_ATOMIC);
	if (!desc)
		return NULL;

	desc->bLength = sizeof(struct usb_interface_descriptor);
	desc->bDescriptorType = USB_DT_INTERFACE;
	desc->bNumEndpoints = src->bNumEndpoints;
	desc->bInterfaceProtocol = src->bInterfaceProtocol;
	desc->bInterfaceClass = src->bInterfaceClass;
	desc->bInterfaceSubClass = src->bInterfaceSubClass;

	return desc;
}

static int usb_bridge_alloc_ep_desc(struct ub_func *f, size_t len)
{
	f->fs_eps = kzalloc(len, GFP_ATOMIC);
	if (!f->fs_eps)
		goto fs_err;

	f->hs_eps = kzalloc(len, GFP_ATOMIC);
	if (!f->hs_eps)
		goto hs_err;

	f->ss_eps = kzalloc(len, GFP_ATOMIC);
	if (!f->ss_eps)
		goto ss_err;

	f->ss_comp_eps = kzalloc(len, GFP_ATOMIC);
	if (!f->ss_comp_eps)
		goto ss_comp_err;

	return 0;

ss_comp_err:
	kfree(f->ss_eps);
ss_err:
	kfree(f->hs_eps);
hs_err:
	kfree(f->fs_eps);
fs_err:
	return -ENOMEM;
}

static int usb_bridge_alloc_desc_hdr(struct ub_func_desc_hdr *hdr,
					       size_t len, size_t len_ss)
{
	hdr->fs = kzalloc(len, GFP_ATOMIC);
	if (hdr->fs == NULL)
		goto fs_err;

	hdr->hs = kzalloc(len, GFP_ATOMIC);
	if (hdr->hs == NULL)
		goto hs_err;

	hdr->ss = kzalloc(len_ss, GFP_ATOMIC);
	if (hdr->ss == NULL)
		goto ss_err;

	return 0;

ss_err:
	kfree(hdr->hs);
hs_err:
	kfree(hdr->fs);
fs_err:
	return -ENOMEM;
}

static uint8_t usb_bridge_alloc_endpoint(struct ub_func *func_data,
					 struct ub_iface *ub_i_data,
					 int desc_index)
{
	int i = 0;
	int ep_count = 0;
	int ss_index = desc_index;
	size_t len = 0;
	struct device *dev = &func_data->config->cdev->gadget->dev;
	struct ub_func_desc_hdr desc_hdr;
	struct usb_descriptor_header *dh = NULL;
	struct usb_endpoint_descriptor *epd = NULL;
	struct usb_composite_dev *cdev = func_data->config->cdev;
	struct ub_func_eps *feps = &func_data->f_i_eps;

	desc_hdr.fs = func_data->fs_func_desc_header;
	desc_hdr.hs = func_data->hs_func_desc_header;
	desc_hdr.ss = func_data->ss_func_desc_header;

	if (ub_i_data->ep_count.bulk_in) {
		/* Allocate Bulk IN EPs */
		len = (sizeof(struct usb_ep *))*ub_i_data->ep_count.bulk_in;
		feps->bulk_in_ep = kmalloc(len, GFP_ATOMIC);
		if (feps->bulk_in_ep == NULL)
			return -ENOMEM;

		for (i = 0; i < ub_i_data->ep_count.bulk_in; i++) {
			usb_bridge_fill_bulk_ep_desc(func_data, ep_count,
						     USB_DIR_IN);

			epd = &func_data->fs_eps[ep_count];
			feps->bulk_in_ep[i] = usb_ep_autoconfig(cdev->gadget,
								epd);

			func_data->hs_eps[ep_count].bEndpointAddress =
				epd->bEndpointAddress;
			func_data->ss_eps[ep_count].bEndpointAddress =
				epd->bEndpointAddress;

			dh = (struct usb_descriptor_header *)epd;
			desc_hdr.fs[desc_index] = dh;

			dh = (struct usb_descriptor_header *)
			      &func_data->hs_eps[ep_count];
			desc_hdr.hs[desc_index] = dh;

			dh = (struct usb_descriptor_header *)
			      &func_data->ss_eps[ep_count];
			desc_hdr.ss[ss_index++] = dh;

			dh = (struct usb_descriptor_header *)
			      &func_data->ss_comp_eps[ep_count];

			desc_hdr.ss[ss_index++] = dh;

			desc_index++;
			ep_count++;
		}
	}

	if (ub_i_data->ep_count.bulk_out) {
		/* Allocate Bulk OUT EPs */
		len = (sizeof(struct usb_ep *))*ub_i_data->ep_count.bulk_out;
		feps->bulk_out_ep = kmalloc(len, GFP_ATOMIC);
		if (feps->bulk_out_ep == NULL)
			goto bout_err;

		for (i = 0; i < ub_i_data->ep_count.bulk_out; i++) {
			usb_bridge_fill_bulk_ep_desc(func_data, ep_count,
						     USB_DIR_OUT);

			epd  = &func_data->fs_eps[ep_count];
			feps->bulk_out_ep[i] = usb_ep_autoconfig(cdev->gadget,
								 epd);

			func_data->hs_eps[ep_count].bEndpointAddress =
				epd->bEndpointAddress;
			func_data->ss_eps[ep_count].bEndpointAddress =
				epd->bEndpointAddress;

			dh = (struct usb_descriptor_header *)epd;
			desc_hdr.fs[desc_index] = dh;

			dh = (struct usb_descriptor_header *)
			      &func_data->hs_eps[ep_count];
			desc_hdr.hs[desc_index] = dh;

			dh = (struct usb_descriptor_header *)
			      &func_data->ss_eps[ep_count];
			desc_hdr.ss[ss_index++] = dh;

			dh = (struct usb_descriptor_header *)
			      &func_data->ss_comp_eps[ep_count];

			desc_hdr.ss[ss_index++] = dh;

			desc_index++;
			ep_count++;
		}
	}

	dev_dbg(dev, "Endpoint alloc success\n");
	return 0;

bout_err:
	kfree(feps->bulk_in_ep);
	return -ENOMEM;
}

static struct ub_req *usb_bridge_req_alloc(struct usb_ep *ep,
					   uint8_t iface_ep_addr,
					   struct ub_func *func_data,
					   int ep_dir, int xfer_type,
					   int interval,
					   gfp_t gfp_flags)
{
	struct ub_req *req = NULL;
	struct device *dev = NULL;
	usb_complete_t complete;
	unsigned int pipe;
	unsigned int length = 0;

	dev = &func_data->config->cdev->gadget->dev;
	req = kzalloc(sizeof(struct ub_req), gfp_flags);
	if (!req)
		return NULL;

	req->ub_f_ep = ep;
	req->ep_dir = ep_dir;

	req->req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req->req)
		goto ep_err;

	if (ep_dir == USB_DIR_IN)
		req->req->complete = usb_bridge_host_in_compl;
	else
		req->req->complete = usb_bridge_host_out_compl;

	length = USB_MAXCONFIG * (unsigned int)le16_to_cpu(USB_DESC_BUFSIZE);

	req->req->length = length;
	req->req->buf = kzalloc(length, gfp_flags);
	if (!req->req->buf)
		goto buf_err;

	req->req->context = req;
	req->req->zero = 0;

	/* Allocate URB */
	req->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!req->urb)
		goto urb_err;

	if (xfer_type == USB_ENDPOINT_XFER_BULK) {
		switch (ep_dir) {
		case USB_DIR_IN:
			pipe = usb_rcvbulkpipe(func_data->iface_data->udev,
					       iface_ep_addr);
			complete = usb_bridge_device_in_compl;
			break;
		case USB_DIR_OUT:
			pipe = usb_sndbulkpipe(func_data->iface_data->udev,
					       iface_ep_addr);
			complete = usb_bridge_device_out_compl;
			break;
		default:
			goto urb_err;
		}
		usb_fill_bulk_urb(req->urb, func_data->iface_data->udev, pipe,
				  req->req->buf, req->req->length,
				  complete, (void *)req);
	} else {
		dev_err(dev, "%s: Invalid transfer type\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	return req;

urb_err:
	kfree(req->req->buf);
buf_err:
	usb_ep_free_request(req->ub_f_ep, req->req);
ep_err:
	kfree(req);
	return ERR_PTR(-ENOMEM);
}

static void usb_bridge_req_free(struct ub_req *req, struct device *dev)
{
	int old_state = 0;

	if (req == NULL) {
		dev_err(dev, "USB request is NULL\n");
		return;
	}

	old_state = req->state;
	req->state = UB_REQ_DO_NOT_SUBMIT;
	usb_unlink_urb(req->urb);
	usb_ep_dequeue(req->ub_f_ep, req->req);

	/* In case a URB is submitted for this request, do not
	 * free the request buffer until the completion callback
	 * for the URB is called. The buffer shall be freed there.
	 *
	 * Safe to free buffer if the request is queued to the ep
	 * as usb_ep_dequeue ensures that the completion callback was
	 * called.
	 */
	if (old_state != UB_REQ_URB_SUBMIT)
		kfree(req->req->buf);

	usb_ep_free_request(req->ub_f_ep, req->req);
	kfree(req);
	req = NULL;
}

static int usb_bridge_do_setup(struct ub_func *func_data, gfp_t gfp_flags)
{
	uint8_t ep_addr;
	int ret = 0;
	int i = 0;
	int bulk_in = 0, bulk_out = 0;
	int nep = func_data->num_ep;
	size_t len = 0;
	struct usb_ep *ep = NULL;
	struct device *dev = NULL;

	dev = &func_data->config->cdev->gadget->dev;
	bulk_in = func_data->f_i_eps.ep_count.bulk_in;
	bulk_out = func_data->f_i_eps.ep_count.bulk_out;

	/* Setup requests (num_ep)*/
	len = (sizeof(struct ub_req *))*nep;
	func_data->ub_reqs = kzalloc(len, gfp_flags);
	if (func_data->ub_reqs == NULL)
		return -ENOMEM;

	/* nep is an index to an array */
	nep--;

	/* Setup Bulk IN requests */
	for (i = 0; i < bulk_in; i++, nep--) {
		ep = func_data->f_i_eps.bulk_in_ep[i];
		ep_addr = func_data->iface_data->b_in_ep[i].bEndpointAddress;
		func_data->ub_reqs[nep] = usb_bridge_req_alloc(ep, ep_addr,
							func_data,
							USB_DIR_IN,
							USB_ENDPOINT_XFER_BULK,
							0, gfp_flags);
		if (IS_ERR(func_data->ub_reqs[nep])) {
			ret = PTR_ERR(func_data->ub_reqs[nep]);
			goto err;
		}

		/* Submit URB to receive data from device */
		ret = usb_submit_urb(func_data->ub_reqs[nep]->urb,
				     GFP_KERNEL);
		if (ret) {
			dev_err(dev, "URB submit error: %d\n", ret);
			goto err;
		}

		func_data->ub_reqs[nep]->state = UB_REQ_URB_SUBMIT;
	}

	/* Setup Bulk OUT requests */
	for (i = 0; i < bulk_out; i++, nep--) {
		ep = func_data->f_i_eps.bulk_out_ep[i];
		ep_addr = func_data->iface_data->b_out_ep[i].bEndpointAddress;
		func_data->ub_reqs[nep] = usb_bridge_req_alloc(ep, ep_addr,
							func_data,
							USB_DIR_OUT,
							USB_ENDPOINT_XFER_BULK,
							0, gfp_flags);
		if (IS_ERR(func_data->ub_reqs[nep])) {
			ret = PTR_ERR(func_data->ub_reqs[nep]);
			goto err;
		}

		/* Queue receive request to the host */
		ret = usb_ep_queue(func_data->ub_reqs[nep]->ub_f_ep,
				   func_data->ub_reqs[nep]->req,
				   GFP_KERNEL);
		if (ret < 0) {
			dev_err(dev, "EP queue failed for intf %d\n",
				func_data->iface_name);
			goto err;
		}

		func_data->ub_reqs[nep]->state = UB_REQ_QUEUE_SUBMIT;
	}

	dev_info(dev, "Interface %d setup\n", func_data->iface_name);

	func_data->enabled = true;
	return 0;

err:
	for (i = 0; i < func_data->num_ep; i++)
		usb_bridge_req_free(func_data->ub_reqs[i], dev);

	return ret;
}

static void usb_bridge_setup(struct work_struct *data)
{
	struct ub_func *func_data = NULL;
	int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&ub_f_setup.wq_lock, flags);
	for (i = 0; i < MAX_CONFIG_INTERFACES; i++) {
		func_data = ub_f_setup.func_data[i];
		if (func_data) {
			spin_unlock_irqrestore(&ub_f_setup.wq_lock, flags);
			usb_bridge_do_setup(func_data, GFP_USER);
			ub_f_setup.func_data[i] = NULL;
			spin_lock_irqsave(&ub_f_setup.wq_lock, flags);
		}
	}
	spin_unlock_irqrestore(&ub_f_setup.wq_lock, flags);
}

static void usb_bridge_teardown(struct ub_func *func_data)
{
	struct ub_req **ub_req = func_data->ub_reqs;
	struct device *dev = &func_data->config->cdev->gadget->dev;
	int i = 0;

	for (i = 0; i < func_data->num_ep; i++)
		usb_bridge_req_free(ub_req[i], dev);

	kfree(func_data->ub_reqs);
	func_data->ub_reqs = NULL;
}

static int usb_bridge_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct ub_func *func_data = NULL;
	struct ub_iface *ub_i_data = NULL;
	struct ub_func_desc_hdr desc_hdr;
	struct usb_interface *ub_i_intf = NULL;
	struct usb_composite_dev *cdev = NULL;
	struct usb_host_interface *cur_altset = NULL;
	struct usb_interface_descriptor *iface_desc = NULL;
	struct usb_interface_descriptor *src_desc = NULL;
	struct usb_interface_assoc_descriptor *assoc_desc = NULL;
	struct device *dev = NULL;
	int index = 0;
	int num_ss_ep = 0;
	int ret = -ENOMEM;
	size_t len = 0;
	size_t len_ss = 0;
	uint8_t num_ep = 0;
	uint8_t num_descs = 0;
	uint8_t func_id = 0;

	func_data = container_of(f, struct ub_func, function);
	func_data->config = c;
	cdev = c->cdev;
	func_id = func_data->iface_name;
	dev = &cdev->gadget->dev;

	ub_i_data = usb_bridge_interface_get(func_id);
	if (!ub_i_data) {
		dev_err(dev, "Interface %d not found\n", func_id);
		ret = -ENODEV;
		goto err;
	}

	if (usb_bridge_interface_claim(ub_i_data)) {
		dev_err(dev, "Unable to claim interface\n");
		ret = -ENXIO;
		goto err;
	}

	func_data->vid = ub_i_data->udev->descriptor.idVendor;
	func_data->pid = ub_i_data->udev->descriptor.idProduct;

	ub_i_intf = ub_i_data->interface;
	cur_altset = ub_i_intf->cur_altsetting;

	/* Allocate Interface Desccriptor */
	src_desc = &cur_altset->desc;
	iface_desc = usb_bridge_alloc_iface_desc(src_desc);
	if (!iface_desc) {
		dev_err(dev, "Unable to allocate interface desc\n");
		goto err;
	}

	/* Get interface id */
	func_id = usb_interface_id(c, f);
	if (func_id < 0) {
		dev_err(dev, "Unable to get interface id\n");
		goto iface_err;
	}

	/* Allocate Interface Association Desccriptor */
	if (ub_i_intf->intf_assoc &&
	    (ub_i_data->intf_id == ub_i_intf->intf_assoc->bFirstInterface)) {
		dev_dbg(dev, "Interface association descriptor available\n");
		assoc_desc = usb_bridge_alloc_assoc_desc(ub_i_intf->intf_assoc);
		if (!assoc_desc) {
			dev_err(dev, "Unable to alloc interface assoc desc\n");
			goto assoc_err;
		}
		assoc_desc->bFirstInterface = func_id;
		num_descs++;
		func_data->assoc_desc = true;
	}

	iface_desc->bInterfaceNumber = func_id;
	func_id = usb_string_id(cdev);
	if (func_id < 0)
		goto str_err;
	func_data->strings[0].id = func_id;
	func_data->strings[0].s = cur_altset->string;
	iface_desc->iInterface = func_id;

	/* Allocate descriptor headers */
	num_ss_ep = (cur_altset->desc.bNumEndpoints*2);
	num_ep = cur_altset->desc.bNumEndpoints;
	num_descs += num_ep + 2;
	len = sizeof(struct usb_descriptor_header *)*num_descs;
	len_ss = sizeof(struct usb_descriptor_header *)*(num_ss_ep+2);
	if (usb_bridge_alloc_desc_hdr(&desc_hdr, len, len_ss)) {
		dev_err(dev, "Unable to allocate descriptor headers\n");
		goto str_err;
	}

	/* Allocate Endpoint descriptors */
	len = sizeof(struct usb_endpoint_descriptor)*num_ep;
	if (usb_bridge_alloc_ep_desc(func_data, len)) {
		dev_err(dev, "Unable to allocate ep descriptors\n");
		goto ep_desc_err;
	}

	/* Add descriptors to descriptor header */
	if (assoc_desc != NULL) {
		desc_hdr.fs[index] =
			(struct usb_descriptor_header *)assoc_desc;
		desc_hdr.hs[index] =
			(struct usb_descriptor_header *)assoc_desc;
		desc_hdr.ss[index] =
			(struct usb_descriptor_header *)assoc_desc;
		index++;
	}
	desc_hdr.fs[index] =
		(struct usb_descriptor_header *)iface_desc;
	desc_hdr.hs[index] =
		(struct usb_descriptor_header *)iface_desc;
	desc_hdr.ss[index] =
		(struct usb_descriptor_header *)iface_desc;
	index++;

	func_data->fs_func_desc_header = desc_hdr.fs;
	func_data->hs_func_desc_header = desc_hdr.hs;
	func_data->ss_func_desc_header = desc_hdr.ss;

	if (usb_bridge_alloc_endpoint(func_data, ub_i_data, index)) {
		dev_err(dev, "Unable to allocate endpoints\n");
		goto ep_err;
	}

	func_data->iface_data = ub_i_data;
	func_data->num_ep = num_ep;

	/* Assign Descriptors */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0))
	if (!usb_assign_descriptors(f, desc_hdr.fs, desc_hdr.hs,
				    desc_hdr.ss)) {
#else
	if (!usb_assign_descriptors(f, desc_hdr.fs, desc_hdr.hs,
				    desc_hdr.ss, NULL)) {
#endif
		func_data->bind = true;
		dev_info(dev, "Bound interface %d\n", ub_i_data->intf_id);
		return 0;
	}

	dev_err(dev, "Descriptor alloc failed for intf: %d\n",
		ub_i_data->intf_id);

ep_err:
	kfree(func_data->ss_comp_eps);
	kfree(func_data->ss_eps);
	kfree(func_data->hs_eps);
	kfree(func_data->fs_eps);
ep_desc_err:
	kfree(desc_hdr.fs);
	kfree(desc_hdr.hs);
	kfree(desc_hdr.ss);
str_err:
	kfree(assoc_desc);
assoc_err:
iface_err:
	kfree(iface_desc);
err:
	return ret;
}

static void usb_bridge_do_unbind(struct usb_function *f)
{
	struct ub_func *func_data = NULL;
	struct device *dev = NULL;
	int i = 0;
	int bulk_in = 0, bulk_out = 0;

	func_data = container_of(f, struct ub_func, function);
	dev = &func_data->config->cdev->gadget->dev;

	bulk_in = func_data->f_i_eps.ep_count.bulk_in;
	bulk_out = func_data->f_i_eps.ep_count.bulk_out;

	usb_free_all_descriptors(f);

	/* Release captured EP's */
	for (i = 0; i < bulk_in; i++)
		usb_ep_autoconfig_release(func_data->f_i_eps.bulk_in_ep[i]);

	for (i = 0; i < bulk_out; i++)
		usb_ep_autoconfig_release(func_data->f_i_eps.bulk_out_ep[i]);

	kfree(func_data->f_i_eps.bulk_in_ep);
	kfree(func_data->f_i_eps.bulk_out_ep);
	kfree(func_data->fs_eps);
	kfree(func_data->hs_eps);
	kfree(func_data->ss_eps);
	kfree(func_data->ss_comp_eps);

	if (func_data->assoc_desc) {
		/* Free the interface descriptor and the association
		 * descriptor
		 */
		kfree(func_data->fs_func_desc_header[0]);
		kfree(func_data->fs_func_desc_header[1]);
		func_data->assoc_desc = false;
	} else {
		kfree(func_data->fs_func_desc_header[0]);
	}

	kfree(func_data->fs_func_desc_header);
	kfree(func_data->hs_func_desc_header);
	kfree(func_data->ss_func_desc_header);

	/* Release claimed interface */
	func_data->bind = false;
	func_data->iface_data = NULL;

	/* Need to check if the function was deactivated
	 * in the disconnect call on the interface side.
	 *
	 * Need to activate this function before unbinding
	 * as the gadget is deactivated when a function deactivates
	 * and the gadget would not reactivate until the function
	 * activates. To avoid the gadget to be deactivated indefinitely,
	 * activate this function if it is deactivated.
	 */
	if (func_data->func_state == UB_FUNC_DEACT)
		usb_function_activate(f);

	dev_info(dev, "Unbound interface %d\n", func_data->iface_name);
}

static void usb_bridge_unbind(struct usb_configuration *c,
			      struct usb_function *f)
{
	usb_bridge_do_unbind(f);
}

static uint8_t usb_bridge_ep_enable(struct ub_func *func_data,
				    struct usb_ep *ep)
{
	struct usb_composite_dev *cdev = func_data->config->cdev;
	struct device *dev = &cdev->gadget->dev;
	uint8_t ret = 0;

	usb_ep_disable(ep);
	ret = config_ep_by_speed(cdev->gadget, &func_data->function, ep);
	if (ret) {
		dev_err(dev, "Unable to configure EP by speed\n");
		return ret;
	}

	ret = usb_ep_enable(ep);
	if (ret) {
		dev_err(dev, "Unable to enable EP\n");
		return ret;
	}

	ep->driver_data = func_data;

	return 0;
}

static int usb_bridge_set_alt(struct usb_function *f, unsigned int intf,
			      unsigned int alt)
{
	struct usb_ep **eps = NULL;
	struct ub_func *func_data = NULL;
	struct ub_ep_count *ep_count = NULL;
	struct device *dev = NULL;
	int error = 0;
	int i = 0;
	unsigned long flags;

	func_data = container_of(f, struct ub_func, function);
	dev = &func_data->config->cdev->gadget->dev;
	ep_count = &func_data->f_i_eps.ep_count;
	ep_count->bulk_in = func_data->iface_data->ep_count.bulk_in;
	ep_count->bulk_out = func_data->iface_data->ep_count.bulk_out;

	if (ep_count->bulk_in) {
		eps = func_data->f_i_eps.bulk_in_ep;
		for (i = 0; i < ep_count->bulk_in; i++) {
			if (usb_bridge_ep_enable(func_data, eps[i]))
				error--;
		}
	}

	if (ep_count->bulk_out) {
		eps = func_data->f_i_eps.bulk_out_ep;
		for (i = 0; i < ep_count->bulk_out; i++) {
			if (usb_bridge_ep_enable(func_data, eps[i]))
				error--;
		}
	}

	if (error) {
		dev_err(dev, "EP enable failed for interface %d\n",
			func_data->iface_name);
		return error;
	}

	/* queue work */
	spin_lock_irqsave(&ub_f_setup.wq_lock, flags);
	ub_f_setup.func_data[func_data->iface_name] = func_data;
	spin_unlock_irqrestore(&ub_f_setup.wq_lock, flags);

	queue_work(ub_f_wq, &ub_f_setup.work);

	return 0;
}

static void usb_bridge_disable(struct usb_function *f)
{
	struct ub_func *func_data = NULL;
	struct ub_ep_count *ep_count = NULL;
	struct device *dev = NULL;
	int i = 0;

	func_data = container_of(f, struct ub_func, function);
	dev = &func_data->config->cdev->gadget->dev;
	ep_count = &func_data->f_i_eps.ep_count;

	if (func_data->enabled) {
		func_data->enabled = false;

		usb_bridge_teardown(func_data);

		for (i = 0; i < ep_count->bulk_in; i++)
			usb_ep_disable(func_data->f_i_eps.bulk_in_ep[i]);

		for (i = 0; i < ep_count->bulk_out; i++)
			usb_ep_disable(func_data->f_i_eps.bulk_out_ep[i]);
	}
	dev_info(dev, "Disabled interface %d\n", func_data->iface_name);
}

static int usb_bridge_set_inst_name(struct usb_function_instance *fi,
					    const char *name)
{
	struct ub_func *func_data = NULL;

	func_data = container_of(fi, struct ub_func, func_inst);

	if (kstrtou8(name, 10, &func_data->iface_name))
		return -EINVAL;

	return 0;
}

static struct configfs_item_operations ub_item_ops = {
	.release = usb_bridge_attr_release,
};

static struct config_item_type ub_func_type = {
	.ct_item_ops = &ub_item_ops,
	.ct_owner = THIS_MODULE,
};

static void usb_bridge_free_func(struct usb_function *f)
{
	struct ub_func *func_data = NULL;

	func_data = container_of(f, struct ub_func, function);

	/* Must free the function but since it is not dynamic */
	func_data->function.name        = NULL;
	func_data->function.bind        = NULL;
	func_data->function.unbind      = NULL;
	func_data->function.set_alt     = NULL;
	func_data->function.disable     = NULL;
	func_data->function.strings     = NULL;
	func_data->function.free_func   = NULL;
}

static struct usb_function *usb_bridge_alloc(struct usb_function_instance *fi)
{
	struct ub_func *func_data = NULL;

	func_data = container_of(fi, struct ub_func, func_inst);

	func_data->function.name        = "USB Bridge";
	func_data->function.bind        = usb_bridge_bind;
	func_data->function.unbind      = usb_bridge_unbind;
	func_data->function.set_alt     = usb_bridge_set_alt;
	func_data->function.disable     = usb_bridge_disable;
	func_data->function.strings     = func_data->strings_list;
	func_data->function.free_func   = usb_bridge_free_func;

	return &func_data->function;
}

static void usb_bridge_free_func_inst(struct usb_function_instance *fi)
{
	struct ub_func *func_data = NULL;
	unsigned long flags;

	func_data = container_of(fi, struct ub_func, func_inst);

	/* Remove function from list */
	spin_lock_irqsave(&ub_func_data.func_lock, flags);
	list_del(&(func_data->func_hook));
	spin_unlock_irqrestore(&ub_func_data.func_lock, flags);
	kfree(func_data->strings_list);
	kfree(func_data);
}

static struct usb_function_instance *usb_bridge_alloc_instance(void)
{
	struct ub_func *func_data = NULL;
	struct usb_gadget_strings **str = NULL;
	unsigned long flags;

	func_data = kzalloc(sizeof(struct ub_func), GFP_KERNEL);
	if (func_data == NULL)
		goto err;

	func_data->func_inst.free_func_inst = usb_bridge_free_func_inst;
	func_data->func_inst.set_inst_name = usb_bridge_set_inst_name;
	str = (struct usb_gadget_strings **)
	       kzalloc(sizeof(struct usb_gadget_strings)*2, GFP_KERNEL);
	if (str == NULL)
		goto err;

	func_data->strings_list = str;
	func_data->stringtab.language = 0x0409; /* en-US */
	func_data->stringtab.strings = &func_data->strings[0];
	func_data->strings_list[0] = &func_data->stringtab;
	func_data->strings_list[1] = NULL;

	config_group_init_type_name(&func_data->func_inst.group, "",
				    &ub_func_type);

	/* Add function to list */
	spin_lock_irqsave(&ub_func_data.func_lock, flags);
	list_add(&(func_data->func_hook), &(ub_func_data.func_list));
	spin_unlock_irqrestore(&ub_func_data.func_lock, flags);

	return &func_data->func_inst;

err:
	kfree(str);
	kfree(func_data);
	return NULL;
}

DECLARE_USB_FUNCTION(usb_bridge, usb_bridge_alloc_instance, usb_bridge_alloc);

void usb_bridge_func_unreg(void)
{
	usb_function_unregister(&usb_bridgeusb_func);
}

struct ub_func_list *usb_bridge_func_init(struct ub_intf_list *ub_intf)
{
	if (ub_intf)
		ub_intf_data = ub_intf;

	if (usb_function_register(&usb_bridgeusb_func))
		return NULL;

	/* Initialize function list */
	INIT_LIST_HEAD(&ub_func_data.func_list);
	spin_lock_init(&ub_func_data.func_lock);

	/* Create workqueue */
	ub_f_wq = create_workqueue("ub_func_wq");
	INIT_WORK(&ub_f_setup.work, usb_bridge_setup);
	spin_lock_init(&ub_f_setup.wq_lock);

	return &ub_func_data;
}
