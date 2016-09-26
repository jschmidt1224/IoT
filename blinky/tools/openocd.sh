#!/bin/bash
__OPENOCD_PATH=/opt/openocd-0.9.0/bin
__OPENOCD_SCRIPTS=$__OPENOCD_PATH/../share/openocd/scripts

cd $__OPENOCD_PATH
sudo ./openocd -f $__OPENOCD_SCRIPTS/board/stm32f4discovery.cfg
