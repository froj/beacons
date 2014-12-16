Beacons
=======
[![Build Status](https://travis-ci.org/cvra/beacons.png)](https://travis-ci.org/cvra/beacons)

# Build
Install the `gcc-arm-none-eabi` toolchain with your favorite package manager.

After you `git clone`d this repo:
```sh
cd beacons
git submodule init
git submodule update
./packager/packager.py
cd libopencm3
make
cd ..
make
```
# Flash
##Install patched version of OpenOCD:

Start by installing `libusb` with your favorite package manager.

Then do:

```sh
git clone https://github.com/cvra/OpenOCD.git
cd OpenOCD
./bootstrap
./configure --enable-ftdi --enable-stlink --enable-buspirate
```

##Connect to the MCU with st-link and flash the .elf:
```sh
sudo openocd -f openocd.cfg
./flash.sh beacons.elf
```
