#!/bin/sh
$DRONE_DIR/test_scripts/takeoff.sh 8
sleep 3
$DRONE_DIR/build/action_goto_nedy --connection_url udp://:14540 --north 0 --east 0 --down -20 --yaw 0 --speed 100
$DRONE_DIR/test_scripts/partner_cycle_shapes.sh &
$DRONE_DIR/build/leader --connection_url udp://:14540 &
