#!/bin/bash

#APP="ukutest.git"
#APP="iperf3"
#APP="cloning-apps"
APP="$1"
PLAT="$2"

# detect platform
if [ -z "$PLAT" ]; then
	if [ -f "/sys/hypervisor/uuid" ]; then
		PLAT="xen"
	else
		PLAT="kvm"
	fi
fi

read_output_xen() {
	#xl dmesg | sed 's/^.*Info: \[libprofiling\] <trace_func.c @  189> \([0-9]\+\.[0-9]\+\) \([0-9]\+\) 0x\([0-9a-f]\+\)/\1 \2 \3/;t;d'
	file="xen.out"
	if [ ! -f $file ]; then
		echo "Cannot find Xen output file: $file"
		exit -2
	fi
	#cat $file | sed 's/^.*Info: \[libprofiling\] <trace_func.c @  189> \([0-9]\+\.[0-9]\+\) \([0-9]\+\) 0x\([0-9a-f]\+\)/\1 \2 \3/;t;d'
	#cat $file | sed 's/^.*Info: \[libprofiling\] <trace_func.c @  199> \([0-9]\+\.[0-9]\+\) \([0-9]\+\.[0-9]\+\) \([0-9]\+\) 0x\([0-9a-f]\+\)/\1 \2 \3 \4/;t;d'
	/home/wolf/nfv/dev/unikraft/unikraft.git/support/scripts/uk_trace/trace.py list --no-tabulate traces.dat.xen | tail -n +3 | tr -s ' ' | cut -d' ' -f3-5
}

read_output_kvm() {
	file="qemu.out"
	if [ ! -f $file ]; then
		echo "Cannot find QEMU output file: $file"
		exit -2
	fi
	#cat $file | sed 's/^.*Info: \[libprofiling\] <trace_func.c @  189> \([0-9]\+\.[0-9]\+\) \([0-9]\+\) 0x\([0-9a-f]\+\)/\1 \2 \3/;t;d'
	#cat $file | sed 's/^.*Info: \[libprofiling\] <trace_func.c @  190> \([0-9]\+\.[0-9]\+\) \([0-9]\+\.[0-9]\+\) \([0-9]\+\) 0x\([0-9a-f]\+\)/\1 \2 \3 \4/;t;d'
	/home/wolf/nfv/dev/unikraft/unikraft.git/support/scripts/uk_trace/trace.py list traces.dat | tail -n +3 | tr -s ' ' | cut -d' ' -f3-5
}

if [ $PLAT = "xen" ]; then
	read_output=read_output_xen
elif [ $PLAT = "kvm" ]; then
	read_output=read_output_kvm
fi

BIN="./build/${APP}_${PLAT}-x86_64.dbg"

#while read start stop duration addr; do
while read start duration addr; do
	val=$(addr2line -fe $BIN $addr)
	func=$(echo $val | cut -d' ' -f1)
	location=$(echo $val | cut -d' ' -f2)
	#echo $location $func $duration
	#echo $start $stop $func $duration
	echo $start $func $duration
done < <($read_output)
