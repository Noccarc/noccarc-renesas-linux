// drivers/misc/nx5_kernel_version.c

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define VERSION "Noccarc PM Kernel v1.2.0"
#define ENTRY_NAME "noccarc_pm_kernel_version"

static int version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", VERSION);
	return 0;
}

static int version_open(struct inode *inode, struct file *file)
{
	return single_open(file, version_show, NULL);
}

static const struct proc_ops version_fops = {
	.proc_open = version_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int __init nx5_version_init(void)
{
	proc_create(ENTRY_NAME, 0, NULL, &version_fops);
	return 0;
}

static void __exit nx5_version_exit(void)
{
	remove_proc_entry(ENTRY_NAME, NULL);
}

module_init(nx5_version_init);
module_exit(nx5_version_exit);

MODULE_LICENSE("GPL");
