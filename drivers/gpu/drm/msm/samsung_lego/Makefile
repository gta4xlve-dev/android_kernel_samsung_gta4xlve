ifeq ($(CONFIG_DISPLAY_SAMSUNG_LEGO),y)
ccflags-y += -I$(srctree)/include/drm
ccflags-y += -I$(srctree)/drivers/gpu/drm/msm
ccflags-y += -I$(srctree)/drivers/gpu/drm/msm/dsi-staging
ccflags-y += -I$(srctree)/drivers/gpu/drm/msm/sde
ccflags-y += -I$(srctree)/drivers/clk/qcom/
ccflags-y += -I$(srctree)/drivers/gpu/drm/msm/samsung_lego
endif

obj-y += ss_dsi_panel_sysfs.o \
	ss_dsi_panel_debug.o \
	ss_dsi_panel_common.o \
	ss_dsi_mdnie_lite_common.o \
	ss_dsi_smart_dimming_common.o \
	ss_interpolation_common.o \
	ss_flash_table_data_common.o \
	ss_dpui_common.o \
	ss_ddi_poc_common.o \
	ss_ddi_spi_common.o \
	ss_copr_common.o

# Tab S6 Lite R panel
obj-$(CONFIG_PANEL_HX83102_TV104WUM_WUXGA) += HX83102_TV104WUM/
