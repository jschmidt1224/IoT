This readme will help you build blinky for the STM32F407VG-Discovery board

You will need to download an ARM toolchain
I suggest gcc-arm-embedded found at:
https://launchpad.net/gcc-arm-embedded
On linux simply download the binary tar.bz2 file and extract to the /opt folder

We use the /opt folder to allow for multiple versions of the toolchain to be
installed simultaneously for different projects
You could also use /usr/local if you prefer

If you choose a different toolchain or install location you will need to modify
the Makefile and envsetup.sh scripts accordingly

Next you will need to install OpenOCD
The source code can be found at:
https://sourceforge.net/projects/openocd/files/openocd/0.9.0/
Download and extract the source code in a location of your choosing

Once extracted cd into the directory and run the following

sudo apt-get install git zlib1g-dev libtool flex bison libgmp3-dev libmpfr-dev libncurses5-dev libmpc-dev autoconf texinfo build-essential libftdi-dev libusb-1.0.0-dev
./configure --prefix=/opt/openocd-0.9.0
make
sudo make install

If you have multiple cores you can run make with the -j n flag to add n threads
to make it compile faster

Again you can choose to install OpenOCD in another location by changing the
prefix when running the ./configure script

Next we must fix an error in the OpenOCD STLINK configure file
Go to the script directory of the openOCD install
For me this is at
/opt/openocd-0.9.0/share/openocd/scripts
cd into the interface directory and modify the file "stlink-v2.cfg" with your
favorite editor

change the line
hla_vid_pid 0x0483 0x3748
to
hla_vid_pid 0x0483 0x374B

The final number '8' should be the letter 'B'

Finally to build and flash

You will want 2 terminal windows to build and flash with ease
In the first run the openocd.sh script in the tools folder
This will ask for your sudo password to allow access to the USB
If you installed OpenOCD in a different location you will need to change the
__OPENOCD_PATH variable in the script

In the second run the envsetup.sh script but be sure to use
. envsetup.sh
and not
./envsetup.sh
because ./ will not actually export to the PATH environment variable

Next simply run
make
make flash

This should erase anything already on the board and install an infinite while
loop that does nothing

Congratulations you just programed the STM32F4-Discovery!
