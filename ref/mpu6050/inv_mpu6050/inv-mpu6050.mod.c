#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

MODULE_INFO(intree, "Y");

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x380122c1, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x7a3cd20d, __VMLINUX_SYMBOL_STR(iio_triggered_buffer_cleanup) },
	{ 0x95373425, __VMLINUX_SYMBOL_STR(iio_trigger_unregister) },
	{ 0xe4689576, __VMLINUX_SYMBOL_STR(ktime_get_with_offset) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x13d0adf7, __VMLINUX_SYMBOL_STR(__kfifo_out) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0xeff07a6f, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x9526bff7, __VMLINUX_SYMBOL_STR(iio_read_const_attr) },
	{ 0xecfb850, __VMLINUX_SYMBOL_STR(i2c_smbus_read_i2c_block_data) },
	{ 0x8ce06f13, __VMLINUX_SYMBOL_STR(iio_trigger_notify_done) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
	{ 0x62b72b0d, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0xc43dc12f, __VMLINUX_SYMBOL_STR(iio_device_register) },
	{ 0xd8e1ec0f, __VMLINUX_SYMBOL_STR(i2c_add_mux_adapter) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0xd697e69a, __VMLINUX_SYMBOL_STR(trace_hardirqs_on) },
	{ 0x17c6b6c5, __VMLINUX_SYMBOL_STR(iio_device_unregister) },
	{ 0xe707d823, __VMLINUX_SYMBOL_STR(__aeabi_uidiv) },
	{ 0xb13c9fd2, __VMLINUX_SYMBOL_STR(devm_iio_trigger_alloc) },
	{ 0x37394e02, __VMLINUX_SYMBOL_STR(devm_iio_device_alloc) },
	{ 0xd6ffa688, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xe16b893b, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x373db350, __VMLINUX_SYMBOL_STR(kstrtoint) },
	{ 0x2196324, __VMLINUX_SYMBOL_STR(__aeabi_idiv) },
	{ 0x75c6e8d7, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xdd5624ca, __VMLINUX_SYMBOL_STR(i2c_del_mux_adapter) },
	{ 0xc6df4828, __VMLINUX_SYMBOL_STR(iio_trigger_register) },
	{ 0x70f680b, __VMLINUX_SYMBOL_STR(get_device) },
	{ 0x59468be9, __VMLINUX_SYMBOL_STR(__module_get) },
	{ 0x63ea111c, __VMLINUX_SYMBOL_STR(__i2c_transfer) },
	{ 0x2d6bcdcb, __VMLINUX_SYMBOL_STR(iio_trigger_generic_data_rdy_poll) },
	{ 0xe2089ba2, __VMLINUX_SYMBOL_STR(iio_push_to_buffers) },
	{ 0xf23fcb99, __VMLINUX_SYMBOL_STR(__kfifo_in) },
	{ 0x7fa21921, __VMLINUX_SYMBOL_STR(i2c_smbus_write_i2c_block_data) },
	{ 0xec3d2e1b, __VMLINUX_SYMBOL_STR(trace_hardirqs_off) },
	{ 0x6cc72d94, __VMLINUX_SYMBOL_STR(devm_request_threaded_irq) },
	{ 0xe847d96f, __VMLINUX_SYMBOL_STR(iio_triggered_buffer_setup) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=industrialio-triggered-buffer,industrialio,i2c-mux";

MODULE_ALIAS("acpi*:INVN6500:*");
MODULE_ALIAS("i2c:mpu6050");
MODULE_ALIAS("i2c:mpu6500");

MODULE_INFO(srcversion, "D44BA4C105265FF9F78A0E7");
