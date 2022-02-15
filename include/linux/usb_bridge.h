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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/usb/composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb.h>
#include <linux/debugfs.h>

#define USB_MAXCONFIG 8
#define USB_DESC_BUFSIZE 512

enum ub_func_state_e {
	UB_FUNC_ACT = 0,
	UB_FUNC_DEACT,
};

enum ub_req_state_e {
	UB_REQ_FREE = 0,
	UB_REQ_URB_SUBMIT,
	UB_REQ_QUEUE_SUBMIT,
	UB_REQ_DO_NOT_SUBMIT,
};

struct ub_req {
	struct urb *urb;
	struct usb_request *req;
	struct usb_ep *ub_f_ep;
	enum ub_req_state_e state;
	int ep_dir;
	unsigned long host_to_ub;
	unsigned long ub_to_device;
	unsigned long device_to_ub;
	unsigned long ub_to_host;
	unsigned long host_to_ub_err;
	unsigned long ub_to_device_err;
	unsigned long device_to_ub_err;
	unsigned long ub_to_host_err;
};

struct ub_ep_count {
	uint8_t bulk_in;
	uint8_t bulk_out;
};

struct ub_func_eps {
	struct usb_ep **bulk_in_ep;
	struct usb_ep **bulk_out_ep;
	struct ub_ep_count ep_count;
};

struct ub_iface {
	struct usb_device *udev;
	struct usb_interface *interface;
	struct usb_endpoint_descriptor b_in_ep[USB_MAXCONFIG];
	struct usb_endpoint_descriptor b_out_ep[USB_MAXCONFIG];
	struct list_head intf_hook;
	struct ub_ep_count ep_count;
	uint8_t intf_id;
};

struct ub_func {
	struct usb_function function;
	struct usb_function_instance func_inst;
	struct usb_configuration *config;
	struct usb_descriptor_header **fs_func_desc_header;
	struct usb_descriptor_header **hs_func_desc_header;
	struct usb_descriptor_header **ss_func_desc_header;
	struct usb_endpoint_descriptor *fs_eps;
	struct usb_endpoint_descriptor *hs_eps;
	struct usb_endpoint_descriptor *ss_eps;
	struct usb_ss_ep_comp_descriptor *ss_comp_eps;
	struct usb_gadget_strings **strings_list;
	struct usb_gadget_strings stringtab;
	struct usb_string strings[2];
	struct list_head func_hook;
	struct ub_func_eps f_i_eps;
	struct ub_req **ub_reqs;
	struct ub_iface *iface_data;
	uint16_t vid;
	uint16_t pid;
	uint8_t num_ep;
	uint8_t iface_name;
	uint8_t assoc_desc;
	uint8_t bind;
	uint8_t enabled;
	uint8_t func_state;
};

struct ub_func_list {
	spinlock_t func_lock;
	struct list_head func_list;
};

struct ub_intf_list {
	spinlock_t intf_lock;
	struct list_head intf_list;	/* List of interfaces */
	bool func_register;
	struct dentry *dentry_root;
};

/*
 * usb_bridge_device_in_compl -IN urb completion callback.
 * @urb: Pointer to the urb that completed
 *
 * Pass the request corresponding to the urb to the host.
 */
void usb_bridge_device_in_compl(struct urb *urb);

/*
 * usb_bridge_device_out_compl -OUT urb completion callback.
 * @urb: Pointer to the urb that completed
 *
 * Queue the request to the host to get the next buffer.
 */
void usb_bridge_device_out_compl(struct urb *urb);

/*
 * usb_bridge_interface_release - Request Interface release.
 * @interface: Pointer to the interface to be released
 */
void usb_bridge_interface_release(struct usb_interface *interface);

/*
 * usb_bridge_interface_claim - Request to claim Interface.
 * @ub_i_data: Pointer to interface instance driver data
 */
int usb_bridge_interface_claim(struct ub_iface *ub_i_data);

/*
 * usb_bridge_interface_get - Get usb bridge interface instance corresponding
 *			      to the function id.
 * @func_id: Function id to look up.
 */
struct ub_iface *usb_bridge_interface_get(uint8_t func_id);

/*
 * usb_bridge_func_unreg - Unregister usb bridge function driver.
 */
void usb_bridge_func_unreg(void);

/*
 * usb_bridge_func_init - Register usb bridge function driver.
 * @ub_intf: Pointer to head of interface list maintained by te driver.
 */
struct ub_func_list *usb_bridge_func_init(struct ub_intf_list *ub_intf);
