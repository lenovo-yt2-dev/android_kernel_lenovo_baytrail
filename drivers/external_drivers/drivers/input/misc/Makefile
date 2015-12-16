ccflags-y += -Werror -Wno-unused-function

intel-mid-vibra-objs := intel_mid_vibra.o
intel-mid-vibra_pf450cl-objs := intel_mid_vibra_me372cl.o intel_mid_vibra_acpi_me372cl.o
obj-$(CONFIG_INPUT_INTEL_MID_VIBRA)		+= intel-mid-vibra.o
obj-$(CONFIG_INPUT_INTEL_MID_VIBRA_PF450CL)     += intel-mid-vibra_pf450cl.o

obj-$(CONFIG_INPUT_HALL)      += hall_switch.o
obj-$(CONFIG_INPUT_M1120)     += m1120.o
