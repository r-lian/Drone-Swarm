#!/bin/sh

echo "Running this script will clear the device passed to this script."
echo "Are you sure you want to continue? (Y/n)"
read continue
if [ $continue = "n" ]
then
    exit 0
fi

sudo parted --align=optimal --script $1 \
    mklabel msdos \
    mkpart primary fat32 0% 200MiB \
    mkpart primary ext4 200MiB 100%

# Create and mount the FAT filesystem:
sudo mkfs.vfat $11
mkdir boot
sudo mount $11 boot

# Create and mount the ext4 filesystem: (#long pause here!)
sudo mkfs.ext4 $12
mkdir root
sudo mount $12 root

# Download and extract the root filesystem:
wget http://os.archlinuxarm.org/os/ArchLinuxARM-rpi-aarch64-latest.tar.gz
sudo bsdtar -xpf ArchLinuxARM-rpi-aarch64-latest.tar.gz -C root
sync

# Move boot files to the first partition:
sudo mv root/boot/* boot

# Unmount the two partitions:
sudo umount boot root

rm -r boot root ArchLinuxARM-rpi-aarch64-latest.tar.gz

echo "The SD card is now ready. Continue following the instructions in the README, including running arch_pi_setup.pi on the booted Rasbperry Pi."
