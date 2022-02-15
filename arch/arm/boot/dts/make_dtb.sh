#!/bin/sh

if ! [ $1 ] ; then
	echo "Please input device name : arch/arm/boot/dts/*.dts"
	echo "ex) $0 imx6q-goembed-b3"
	exit
fi

echo "----compiling $1 device tree------"

IDE=$1
SRC=$IDE.dts
TMP=$IDE.tmp.dts
DST=$IDE.dtb
cpp -nostdinc -I include -undef -x assembler-with-cpp $SRC > $TMP
dtc -O dtb -b 0 -o $DST $TMP
rm $TMP
