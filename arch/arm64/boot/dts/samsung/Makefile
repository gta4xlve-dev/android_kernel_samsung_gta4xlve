ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

# GTA4XLVE EUR
SEC_GTA4XLVE_EUR_DTBO := \
		atoll-sec-gta4xlve-eur-overlay-r00.dtbo

define __sec_dtbo_build
dtbo-$(2) += $(1)
$(1)-base := $(3)
endef

define sec_dtbo_build
$(foreach dtbo, $(1), $(eval $(call __sec_dtbo_build, $(dtbo),$(2),$(3))))
endef

# SM7125 BASE DTB
SEC_ATOLL_BASE_DTB := ../qcom/atoll-ab-idp.dtb

$(eval $(call sec_dtbo_build, \
		$(SEC_GTA4XLVE_EUR_DTBO),$(CONFIG_SEC_GTA4XLVE_EUR_OPEN),$(SEC_ATOLL_BASE_DTB)))

endif

ifeq ($(CONFIG_ARM64),y)
always          := $(dtb-y)
subdir-y        := $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
        $(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))
endif

clean-files := *.dtb *.reverse.dts *.dtbo
