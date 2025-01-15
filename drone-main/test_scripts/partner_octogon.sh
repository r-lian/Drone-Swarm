#!/bin/sh

#s=4->h=2.82

$DRONE_DIR/build/partner --connection_url udp://:14541 --north 4 --east 0 --down 0 --yaw 0 &
$DRONE_DIR/build/partner --connection_url udp://:14542 --north 6.82 --east 2.82 --down 0 --yaw 0 &
$DRONE_DIR/build/partner --connection_url udp://:14543 --north 6.82 --east 6.82 --down 0 --yaw 0 &
$DRONE_DIR/build/partner --connection_url udp://:14544 --north 4 --east 9.64 --down 0 --yaw 0 &
$DRONE_DIR/build/partner --connection_url udp://:14545 --north 0 --east 9.64 --down 0 --yaw 0 &
$DRONE_DIR/build/partner --connection_url udp://:14546 --north -4 --east 2.82 --down 0 --yaw 0 &
$DRONE_DIR/build/partner --connection_url udp://:14547 --north -4 --east 6.82 --down 0 --yaw 0 &
