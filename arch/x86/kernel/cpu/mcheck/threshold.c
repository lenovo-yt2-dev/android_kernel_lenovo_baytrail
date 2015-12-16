/*
 * Common corrected MCE threshold handler code:
 */
#include <linux/interrupt.h>
#include <linux/kernel.h>

#include <asm/irq_vectors.h>
#include <asm/apic.h>
#include <asm/idle.h>
#include <asm/mce.h>

void get_mce_status()
{
	int banks = 0;
	u64 cap = 0;
	u64 val = 0;
	int i = 0;

	printk("Enter:%s", __func__);

	rdmsrl(MSR_IA32_MCG_CAP, cap);
	banks = min_t(unsigned, MAX_NR_BANKS, cap & 0xff);
	printk("banks:%0x, cap:%0x\n", banks, cap);
	for (i = 0; i < banks; i++) {

		rdmsrl(MSR_IA32_MCx_STATUS(i), val);
		if(val & 0x8000000000000000) { //64bit
			printk("i = %d, status: val:%lx ", i, val);
			if(val & 0x0400000000000000) { //58bit
				rdmsrl(MSR_IA32_MCx_ADDR(i), val);
				printk("i=%d, addr: val:%lx", i, val);
			}
			if(val & 0x0800000000000000) { //58bit
				rdmsrl(MSR_IA32_MCx_MISC(i), val);
				printk("i=%d, misc: val:%lx ", i, val);
			}
		}
	}
	rdmsrl(MSR_IA32_MCG_STATUS, val);
	printk("mcg_status:%0x\n", val);
	printk("\n");
}

static void default_threshold_interrupt(void)
{
	printk(KERN_ERR "Unexpected threshold interrupt at vector %x\n",
			 THRESHOLD_APIC_VECTOR);

	get_mce_status();
}

void (*mce_threshold_vector)(void) = default_threshold_interrupt;

asmlinkage void smp_threshold_interrupt(void)
{
	irq_enter();
	exit_idle();
	inc_irq_stat(irq_threshold_count);
	mce_threshold_vector();
	irq_exit();
	/* Ack only at the end to avoid potential reentry */
	ack_APIC_irq();
}
