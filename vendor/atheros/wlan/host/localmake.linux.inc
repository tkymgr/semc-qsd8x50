#
#  Local Makefile includes for tool and kernel source paths
#  ***** This is only a sample, modify the paths for your specific build environment *****
# 
#  Copyright 2004-2006 Atheros Communications, Inc.
#
# When creating a new build type, use the following template:
#  	ifeq ($(ATH_BUILD_TYPE),<platform name>) 
#       ATH_ARCH_CPU_TYPE :=  < cpu architecture >
#       ATH_CROSS_COMPILE_TYPE := < cross compiler path >
#       ATH_LINUXPATH := < kernel source path >
ATH_BUILD_TYPE=QUALCOMM_ARM_NATIVEMMC
_BSP_REV=3240
ATH_BSP_TYPE=QSD8K_BSP$(_BSP_REV)
ATH_ANDROID_ROOT=/opt/qc8k



  $(info *************************)
  $(info PLATFORM: $(ATH_BUILD_TYPE))
  $(info BSP:      $(ATH_BSP_TYPE))
  $(info *************************)

  # Some environment settings
  ATH_BUILD_SYSTEM_TOOLS:= no
  ATH_ANDROID_ENV := yes
  ATH_EEPROM_FILE_USED := no 


  # Some common variable
  ATH_ARCH_CPU_TYPE := arm 
  ATH_CROSS_COMPILE_TYPE := arm-eabi-
  ATH_BUS_TYPE := SDIO
  ATH_OS_SUB_TYPE := linux_2_6
  ATH_BUS_SUBTYPE := linux_sdio

#ATHENV
