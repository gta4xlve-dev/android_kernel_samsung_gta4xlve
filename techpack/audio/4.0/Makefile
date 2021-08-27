ifeq ($(CONFIG_ARCH_ATOLL), y)
ifeq ($(PROJECT_NAME),$(filter $(PROJECT_NAME),gta4xlve gta4xlvewifi))
include $(srctree)/techpack/audio/4.0/config/sm7125_gta4xlve.conf
else
include $(srctree)/techpack/audio/4.0/config/atollauto.conf
endif
export

# Use USERINCLUDE when you must reference the UAPI directories only.
USERINCLUDE     += \
                -I$(srctree)/techpack/audio/4.0/include/uapi \

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/techpack/audio/4.0/include/uapi \
                -I$(srctree)/techpack/audio/4.0/include
ifeq ($(PROJECT_NAME),$(filter $(PROJECT_NAME),gta4xlve gta4xlvewifi))
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/4.0/config/sm7125_gta4xlve.h
else
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/4.0/config/atollautoconf.h
endif
endif

obj-y += soc/
obj-y += dsp/
obj-y += ipc/
obj-y += asoc/
