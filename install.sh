#!/bin/bash

device="zjh_mpu"
module="mpu_module.ko"

/sbin/insmod ./$module || exit 1

rm -f /dev/mpu

major=$(awk '$2=="zjh_mpu" {print $1}' /proc/devices)

echo major=$major

mknod /dev/mpu c $major 0


