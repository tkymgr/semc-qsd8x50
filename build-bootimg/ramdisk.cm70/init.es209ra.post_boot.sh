#!/system/bin/sh
# Copyright (C) 2009 Sony Ericsson Mobile Communications Japan, Inc.
#
# This file is made from init.qcom.sh, and modified for es209ra.
#
# Copyright (c) 2009, Code Aurora Forum. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Code Aurora nor
#       the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior written
#       permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# echo "ondemand" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
# echo 95 > /sys/devices/system/cpu/cpu0/cpufreq/ondemand/up_threshold
# echo 30 > /sys/devices/system/cpu/cpu0/cpufreq/ondemand/down_differential
# echo 500000 > /sys/devices/system/cpu/cpu0/cpufreq/ondemand/sampling_rate
# echo 0 > /sys/devices/system/cpu/cpu0/cpufreq/ondemand/powersave_bias

# create swapfile
# losetup /dev/block/loop7 /swapfile
# mkswap /dev/block/loop7
# swapon /dev/block/loop7

insmod /system/lib/modules/ramzswap.ko
rzscontrol /dev/block/ramzswap0 --init
swapon /dev/block/ramzswap0

