#SPDX-License-Identifier: GPL-2.0-only
DISPLAY_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	ifeq ($(TARGET_KERNEL_DLKM_DISPLAY_OVERRIDE), false)
		DISPLAY_DLKM_ENABLE := false
	endif
endif

ifeq ($(DISPLAY_DLKM_ENABLE),  true)
	ifeq ($(call is-board-platform-in-list,$(TARGET_BOARD_PLATFORM)),true)
		BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/msm_drm.ko \
										$(KERNEL_MODULES_OUT)/ktz8866_leds.ko
		BOARD_VENDOR_RAMDISK_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/msm_drm.ko \
										$(KERNEL_MODULES_OUT)/ktz8866_leds.ko
		BOARD_VENDOR_RAMDISK_RECOVERY_KERNEL_MODULES_LOAD += $(KERNEL_MODULES_OUT)/msm_drm.ko \
										$(KERNEL_MODULES_OUT)/ktz8866_leds.ko
	endif
endif
