#!/bin/sh

sudo pacman -Syu

git clone https://aur.archlinux.org/yay.git
cd yay
makepkg -si
cd ..
rm -r yay

sudo pacman -Syu --noconfirm base-dev gvim NetworkManager firefox dmenu git konsole xorg-server xorg-xinit i3-gaps alsa-utils
yay -S --noconfirm tbsm fake-hwclock mavsdk batctl
