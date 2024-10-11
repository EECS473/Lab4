// This file contains snippets of code that can be added to partC.c //

long memory_ioctl (struct file *filp, unsigned int cmd, \
		  unsigned long arg);

struct file_operations memory_fops = 
{
  .read = memory_read,
  .write = memory_write,
  .open = memory_open,
  .release = memory_release,
  .unlocked_ioctl = memory_ioctl
};

long 
memory_ioctl (struct file *filp, unsigned int cmd,
 	      unsigned long arg)
{
  printk("<1>in ioctl\n");
  if(cmd==0) 
	*memory_buffer^=0x20;	
  else
        *memory_buffer|=0x20;
  return(0); // success!
}

