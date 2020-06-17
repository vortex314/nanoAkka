#!/bin/bash
gpio mode 29 out
gpio write 29 1
#cd /mnt/home/lieven/workspace/esp32-streams
baudrate=921600
export TTY=USB0
while getopts ":x:T:" opt; do
  case $opt in
    x) arg_x="$OPTARG"
    ;;
    T) TTY="$OPTARG"
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
    ;;
  esac
done
SERIAL_PORT="/dev/tty"$TTY
esptool.py --chip esp32 --port $SERIAL_PORT --baud $baudrate \
	--before default_reset --after hard_reset write_flash -z \
	--flash_mode dio --flash_freq 40m --flash_size detect \
	0x1000 build/bootloader/bootloader.bin \
	0x10000 build/nanoAkka.bin \
	0x8000 build/partitions_singleapp.bin
minicom -D /dev/tty$TTY -C /tmp/$TTY_minicom.log

