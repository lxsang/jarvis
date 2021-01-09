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

#ifdef RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x59253af, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x7d34cd8b, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0xa6105046, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xea421b3b, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x60bef4d1, __VMLINUX_SYMBOL_STR(of_property_read_variable_u32_array) },
	{ 0x1a9ced84, __VMLINUX_SYMBOL_STR(of_get_next_child) },
	{ 0xc7190a68, __VMLINUX_SYMBOL_STR(hwmon_device_register) },
	{ 0x7f0bf186, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0xbe55dad3, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xf4e13e7b, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0xe17e2e72, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0xa30ccce8, __VMLINUX_SYMBOL_STR(i2c_smbus_write_word_data) },
	{ 0xf5009364, __VMLINUX_SYMBOL_STR(i2c_smbus_read_word_data) },
	{ 0x531537e, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x1429df6b, __VMLINUX_SYMBOL_STR(device_remove_file) },
	{ 0x2ac7d172, __VMLINUX_SYMBOL_STR(hwmon_device_unregister) },
	{ 0x1fdc7df2, __VMLINUX_SYMBOL_STR(_mcount) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("i2c:ads1015");
MODULE_ALIAS("i2c:ads1115");
