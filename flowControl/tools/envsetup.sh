#!/bin/bash
#
__TOOLS_DIR=$( dirname $( readlink -e ${BASH_SOURCE[0]} ) )
__BASE_DIR=$( dirname ${__TOOLS_DIR} )
#__BASE_DIR=/home/jason/Desktop/blinky/tools
# libc, libncurses 32bit versions are installed
# Qrem why do i care??

# adao recommended using __BASE_DIR=directory of makefile
export PATH=/opt/gcc-arm-none-eabi-5_4-2016q2/bin:$PATH


alias base="cd ${__BASE_DIR}"

function m() {
    make --no-print-directory -C ${__BASE_DIR} $*
}
