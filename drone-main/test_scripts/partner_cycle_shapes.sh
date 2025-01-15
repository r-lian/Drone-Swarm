#!/bin/sh

partner_scripts="$DRONE_DIR/test_scripts/partner_horizontal_line.sh $DRONE_DIR/test_scripts/partner_vertical_line.sh $DRONE_DIR/test_scripts/partner_cube.sh $DRONE_DIR/test_scripts/partner_octogon.sh $DRONE_DIR/test_scripts/partner_pyramid.sh"
# partner_scripts="$DRONE_DIR/test_scripts/partner_horizontal_line.sh $DRONE_DIR/test_scripts/partner_vertical_line.sh"
partner_exe="$DRONE_DIR/build/partner"
echo "--------------------------------------------------"
echo $partner_scripts
while :
do
	for partner_script in $partner_scripts; do
		echo "executing $partner_script"
		# sleep 10
		$partner_script &

		sleep 20
		# pkill $partner_exe
		# echo "killed threads"
		partner_pid_list=$(ps -aux | pgrep "^partner$")
		echo "partner_pid_list: $partner_pid_list"
		echo $partner_pid_list > partner_pid_list.txt
		for partner_pid in $partner_pid_list; do
			kill $partner_pid
		done
	done
done
