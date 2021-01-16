## Prepare

Get the public_sources.tbz2 for jetson nano from NVDIA download page

```sh
mkdir ~/jetson
cp public_sources.tbz2 ~/jetson
cd ~/jetson
tar xf public_sources.tbz2
cd Linux_for_Tegra/source/public/
tar xf kernel_src.tbz2
cd kernel/kernel-4.9/
cat /proc/config.gz | gunzip > .config
cp /usr/src/linux-headers-4.9.140-tegra-linux_x86_64/kernel-4.9/Module.symvers .
make scripts prepare modules_prepare
```

## Build

```sh
cd /path/to/ads1115-driver
KERNEL_SRC=~/jetson/Linux_for_Tegra/source/public/kernel/kernel-4.9 make
```

# install dtbo + module to the system

```sh
sudo cp ads1115-overlay.dtbo /boot/
#  sudo /opt/nvidia/jetson-io/config-by-hardware.py -l
sudo /opt/nvidia/jetson-io/config-by-hardware.py -n "ads1115 Overlay"

cp ads1015.ko /lib/modules/4.9.140-tegra/kernel/drivers/i2c/
sudo depmod
sudo vi /etc/modules
# add ads1015 to the file
```