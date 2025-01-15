#!/bin/sh

# We should check for any necessary programs and install them (or ask if the user wants us to)
# needed programs I can think of so far: parted, wget

#for later, so that every OS can play:
# maybe "lsb_release -d|awk '{print $2}'" instead?
#if [["lsb_release -is" == "Ubuntu"]]; then
#echo("sudo apt-get install parted");
#elif [["lsb_release -is" == "Debian"]]; then
#echo("sudo apt-get install parted");
#elif [["lsb_release -is"  == "RHEL"]]; then
#echo("yum install parted");
#elif [["lsb_release -is"  == "CentOS"]]; then
#echo("yum install parted");
#elif [["lsb_release -is" == "Fedora"]]; then
#echo("dnf install parted");
#else
#echo("Linux distro unrecognised");
#fi

echo "Running this script will clear the device passed to this script."
echo "Are you sure you want to continue? (Y/n)"
read continue
if [ $continue = "n" ]
then
    exit 0
fi

sudo parted --script $1 \
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
wget http://os.archlinuxarm.org/os/ArchLinuxARM-rpi-2-latest.tar.gz
sudo bsdtar -xpf ArchLinuxARM-rpi-2-latest.tar.gz -C root
sync

# Move boot files to the first partition:
sudo mv root/boot/* boot

# Unmount the two partitions:
sudo umount boot root

# Cleaning up working directory
rm -r root boot ArchLinuxARM-rpi-2-latest.tar.gz

echo "Arch ARM has now be installed onto the SD card. Please remove it, insert it into the Raspberry Pi and complete setup there."
