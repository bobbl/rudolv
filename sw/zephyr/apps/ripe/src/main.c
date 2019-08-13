#include <zephyr.h>
#include <sys/printk.h>

void main(void)
{
	printk("Hello from Thales! %s\n", CONFIG_BOARD);
}
