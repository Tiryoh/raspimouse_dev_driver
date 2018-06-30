#!/usr/bin/env bash

set -eu

[[ -z $(which i2cget)  ]] && echo "i2ctools are required" && exit 1

if [ $# -ne 2  ]; then
	echo "Left Motor status"
	i2cget -y 1 0x10 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	echo "Right Motor status"
	i2cget -y 1 0x11 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
else
	if [ $1 -eq "0"  ]; then
		echo "Set Left Motor Counter 0"
		sudo i2cset -y 1 0x10 0x10 0x00 0x00 i
		sleep 1
		echo "Left Motor status"
		i2cget -y 1 0x10 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	fi
	if [ $2 -eq "0"  ]; then
		echo "Set Right Motor Counter 0"
		sudo i2cset -y 1 0x11 0x10 0x00 0x00 i
		sleep 1
		echo "Right Motor status"
		i2cget -y 1 0x11 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	fi
	if [ $1 -eq "123"  ]; then
		echo "Set Left Motor Counter 123"
		sudo i2cset -y 1 0x10 0x10 0x00 0x7B i
		sleep 1
		echo "Left Motor status"
		i2cget -y 1 0x10 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	fi
	if [ $2 -eq "234"  ]; then
		echo "Set Right Motor Counter 234"
		sudo i2cset -y 1 0x11 0x10 0x00 0xEA i
		sleep 1
		echo "Right Motor status"
		i2cget -y 1 0x11 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	fi
fi
