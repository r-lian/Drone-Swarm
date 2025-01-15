#!/bin/sh
# valgrind -s --leak-check=full $DRONE_DIR/build/leader --connection_url udp://:14540
$DRONE_DIR/build/leader --connection_url udp://:14540
