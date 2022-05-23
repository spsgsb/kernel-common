
targets += dtbs

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

DTBO_NAMES := $(subst $\",,$(addsuffix _overlays,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES)))
ifneq ($(DTBO_NAMES),)
DTB_LIST := $(addsuffix .dtbo,$(DTBO_NAMES))
endif

targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := *.dtb *.dtbo
