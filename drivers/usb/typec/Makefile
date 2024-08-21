obj-$(CONFIG_TYPEC)		+= typec.o
obj-$(CONFIG_TYPEC_WCOVE)	+= typec_wcove.o
obj-$(CONFIG_TYPEC_UCSI)	+= ucsi/

ifeq ($(CONFIG_USE_PDIC_LEGO), y)
	obj-$(CONFIG_USB_TYPEC_MANAGER_NOTIFIER)		+= manager_lego/
	obj-$(CONFIG_PDIC_NOTIFIER)		+= common/
	obj-$(CONFIG_PDIC_SM5714)		+= sm/sm5714/
endif
