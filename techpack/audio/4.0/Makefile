ifeq ($(CONFIG_ARCH_ATOLL), y)
ifeq ($(PROJECT_NAME),$(filter $(PROJECT_NAME),gta4xlve gta4xlvewifi))
include $(srctree)/techpack/audio/4.0/config/sm7125_gta4xlve.conf
LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/4.0/config/sm7125_gta4xlve.h
else
include $(srctree)/techpack/audio/4.0/config/atollauto.conf

LINUXINCLUDE    += \
                -include $(srctree)/techpack/audio/4.0/config/atollautoconf.h
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
endif

obj-y += soc/
obj-y += dsp/
obj-y += ipc/
obj-y += asoc/
