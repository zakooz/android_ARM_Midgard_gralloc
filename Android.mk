# 
# Copyright (C) 2010 ARM Limited. All rights reserved.
# 
# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked and stored in
# hw/<OVERLAY_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
include $(BUILD_SYSTEM)/version_defaults.mk
LOCAL_PRELINK_MODULE := false

ifeq ($(MALI_ION), 1)
ALLOCATION_LIB := libion
ALLOCATOR_SPECIFIC_FILES := alloc_ion.cpp gralloc_module_ion.cpp
else
ALLOCATION_LIB := libGLES_mali
ALLOCATOR_SPECIFIC_FILES := alloc_ump.cpp gralloc_module_ump.cpp
endif

LOCAL_SHARED_LIBRARIES := libhardware liblog libcutils libGLESv1_CM $(ALLOCATION_LIB)
LOCAL_C_INCLUDES := $(MALI_LOCAL_PATH) $(MALI_LOCAL_PATH)/kernel/include $(MALI_LOCAL_PATH)/include $(MALI_LOCAL_PATH)/src/ump/include system/core/include/
LOCAL_CFLAGS := -DLOG_TAG=\"gralloc\" -DSTANDARD_LINUX_SCREEN -DMALI_ION=$(MALI_ION) -DGRALLOC_16_BITS -DPLATFORM_SDK_VERSION=$(PLATFORM_SDK_VERSION)

ifeq ($(TARGET_BOARD_PLATFORM),)
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_MODULE := gralloc.default
else
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR_SHARED_LIBRARIES)/hw
LOCAL_MODULE := gralloc.$(TARGET_BOARD_PLATFORM)
endif
LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := \
	gralloc_module.cpp \
	alloc_device.cpp \
	$(ALLOCATOR_SPECIFIC_FILES) \
	framebuffer_device.cpp

include $(BUILD_SHARED_LIBRARY)
