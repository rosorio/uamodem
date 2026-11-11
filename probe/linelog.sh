#!/bin/sh
# Log, once per second for $1 seconds: min/max line voltage (reg 29, 10 samples),
# reg 12 (FDT/LCS) and the number of RING_DETECT notifications in that second.
P=$(dirname "$0")/modemprobe
secs=${1:-180}
ops=""
for i in 1 2 3 4 5 6 7 8 9 10; do ops="$ops x:801d00 s40 r"; done
ops="$ops x:800c00 s40 r"
i=0
while [ $i -lt "$secs" ]; do
	out=$($P seq $ops 2>/dev/null)
	rings=$(echo "$out" | grep -c ' 09 00 00')
	vals=$(echo "$out" | grep 'req=01' | awk '{print $NF}')
	r12=$(echo "$vals" | tail -1)
	echo "$vals" | head -10 | {
		min=999; max=-999
		while read h; do
			v=$((0x$h)); [ $v -gt 127 ] && v=$((v-256))
			[ $v -lt $min ] && min=$v; [ $v -gt $max ] && max=$v
		done
		printf '%s LVS[min=%4d max=%4d] r12=%s rings=%d\n' "$(date +%T)" $min $max "$r12" "$rings"
	}
	i=$((i+1))
done
