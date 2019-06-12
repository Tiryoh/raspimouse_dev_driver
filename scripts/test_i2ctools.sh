#!/usr/bin/env bash

set -eu

[[ -z $(which i2cget) ]] && echo "i2ctools are required" && exit 1

if [ $# -ne 2 ]; then
	echo "Left Motor status"
	i2cget -y 1 0x10 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	echo "Right Motor status"
	i2cget -y 1 0x11 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
else
	if [[ $1 =~ ^[0-9]+$ ]]; then
		echo "Set Left Motor Counter $1"
		i2cset -y 1 0x10 0x10 $(printf '%04x\n' $1 | sed 's/\(..\)\(..\)/0x\1 0x\2/g') i
		echo "Left Motor status"
		i2cget -y 1 0x10 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	fi
	if [[ $2 =~ ^[0-9]+$ ]]; then
		echo "Set Right Motor Counter $2"
		i2cset -y 1 0x11 0x10 $(printf '%04x\n' $2 | sed 's/\(..\)\(..\)/0x\1 0x\2/g') i
		echo "Right Motor status"
		i2cget -y 1 0x11 0x10 w | sed 's/x\(..\)\(..\)/x\2\1/g'
	fi
fi
