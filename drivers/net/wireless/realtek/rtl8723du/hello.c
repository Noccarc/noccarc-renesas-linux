#include "linux/module.h"
//#include <linux/kernel.h>
static int __init helloworld_init(void)
{
    pr_info("Hello world \n");
    return 0;    

}

static void __exit helloworld_cleanup(void)
{
   pr_info("Good by world \n");
}

module_init(helloworld_init);
module_exit(helloworld_cleanup);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("SAMAR");
MODULE_DESCRIPTION("A simple hello world kernal module");
MODULE_INFO(board,"renesas kernal driver");

