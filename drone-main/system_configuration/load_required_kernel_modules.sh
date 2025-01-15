#!/bin/sh

cat /etc/modules-load.d/raspberrypi.conf
echo "batman_adv" >> /etc/modules-load.d/raspberrypi.conf
modprobe batman_adv
