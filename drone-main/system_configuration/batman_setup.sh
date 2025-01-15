#!/bin/sh

iw dev wlan0 del
iw phy phy0 interface add wlan0 type ibss
ip link set up mtu 1532 dev wlan0
iw dev wlan0 ibss join my-mesh-network 2412 HT20 fixed-freq 02:12:34:56:78:9A

batctl if add wlan0
ip link set up dev bat0
