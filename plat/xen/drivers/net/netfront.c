/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Costin Lupu <costin.lupu@cs.pub.ro>
 *          Razvan Cojocaru <razvan.cojocaru93@gmail.com>
 *
 * Copyright (c) 2020, University Politehnica of Bucharest. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * THIS HEADER MAY NOT BE EXTRACTED OR MODIFIED IN ANY WAY.
 */

#include <uk/assert.h>
#include <uk/print.h>
#include <uk/alloc.h>
#include <uk/netdev_driver.h>
#include <xenbus/xenbus.h>
#include "netfront.h"
#include "netfront_xb.h"


#define DRIVER_NAME  "xen-netfront"

#define to_netfront_dev(dev) \
	__containerof(dev, struct netfront_dev, netdev)

static struct uk_alloc *drv_allocator;

static const void *netfront_einfo_get(struct uk_netdev *n,
		enum uk_netdev_einfo_type einfo_type)
{
	struct netfront_dev *nfdev;

	UK_ASSERT(n != NULL);

	nfdev = to_netfront_dev(n);
	switch (einfo_type) {
	case UK_NETDEV_IPV4_ADDR_STR:
		return nfdev->econf.ipv4addr;
	case UK_NETDEV_IPV4_MASK_STR:
		return nfdev->econf.ipv4mask;
	case UK_NETDEV_IPV4_GW_STR:
		return nfdev->econf.ipv4gw;
	default:
		break;
	}

	/* type not supported */
	return NULL;
}

static const struct uk_hwaddr *netfront_mac_get(struct uk_netdev *n)
{
	struct netfront_dev *nfdev;

	UK_ASSERT(n != NULL);
	nfdev = to_netfront_dev(n);
	return &nfdev->hw_addr;
}

static uint16_t netfront_mtu_get(struct uk_netdev *n)
{
	struct netfront_dev *nfdev;

	UK_ASSERT(n != NULL);
	nfdev = to_netfront_dev(n);
	return nfdev->mtu;
}

static unsigned int netfront_promisc_get(struct uk_netdev *n)
{
	struct netfront_dev *nfdev;

	UK_ASSERT(n != NULL);
	nfdev = to_netfront_dev(n);
	return nfdev->promisc;
}

static const struct uk_netdev_ops netfront_ops = {
	.einfo_get = netfront_einfo_get,
	.hwaddr_get = netfront_mac_get,
	.mtu_get = netfront_mtu_get,
	.promiscuous_get = netfront_promisc_get,
};

static int netfront_add_dev(struct xenbus_device *xendev)
{
	struct netfront_dev *nfdev;
	int rc = 0;

	UK_ASSERT(xendev != NULL);

	nfdev = uk_calloc(drv_allocator, 1, sizeof(*nfdev));
	if (!nfdev) {
		rc = -ENOMEM;
		goto err_out;
	}

	nfdev->xendev = xendev;
	nfdev->mtu = UK_ETH_PAYLOAD_MAXLEN;

	/* Xenbus initialization */
	rc = netfront_xb_init(nfdev, drv_allocator);
	if (rc) {
		uk_pr_err("Error initializing Xenbus data: %d\n", rc);
		goto err_xenbus;
	}

	/* register netdev */
	nfdev->netdev.ops = &netfront_ops;
	rc = uk_netdev_drv_register(&nfdev->netdev, drv_allocator, DRIVER_NAME);
	if (rc < 0) {
		uk_pr_err("Failed to register %s device with libuknetdev\n",
			DRIVER_NAME);
		goto err_register;
	}
	nfdev->uid = rc;
	rc = 0;

out:
	return rc;
err_register:
	netfront_xb_fini(nfdev, drv_allocator);
err_xenbus:
	uk_free(drv_allocator, nfdev);
err_out:
	goto out;
}

static int netfront_drv_init(struct uk_alloc *allocator)
{
	/* driver initialization */
	if (!allocator)
		return -EINVAL;

	drv_allocator = allocator;
	return 0;
}

static const xenbus_dev_type_t netfront_devtypes[] = {
	xenbus_dev_vif,
	xenbus_dev_none
};

static struct xenbus_driver netfront_driver = {
	.device_types = netfront_devtypes,
	.init         = netfront_drv_init,
	.add_dev      = netfront_add_dev
};
XENBUS_REGISTER_DRIVER(&netfront_driver);
