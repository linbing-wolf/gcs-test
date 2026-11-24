// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/of.h>
#include "overlay_blob.h"

static int overlay_id;
static int __init maix_slave_init(void)
{
    return of_overlay_fdt_apply(overlay_blob, sizeof(overlay_blob), &overlay_id);
}
static void __exit maix_slave_exit(void)
{
    int ret = of_overlay_remove(&overlay_id);
    if (ret)
        pr_err("maix_i2c6_slave: overlay remove failed: %d\n", ret);
}
module_init(maix_slave_init);
module_exit(maix_slave_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Temporary MaixCAM2 I2C6 slave device-tree overlay");
