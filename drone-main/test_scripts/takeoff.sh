#!/bin/sh

declare -i first_port=14540
declare -i last_port=$first_port+$1-1
declare -i last_check=$last_port+1
declare -i port=$first_port
declare -i counter=0

echo $last_port
if [ $last_port -lt 14540 ]; then
	exit
fi

while [ $port -lt $last_check ]; do
	echo $port
	$DRONE_DIR/build/takeoff --connection_url udp://:$port &
	declare -i port=$port+1
done
