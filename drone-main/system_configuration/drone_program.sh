#!/bin/sh

echo "drone dir $DRONE_DIR"
DEFAULT_PROJ=$DRONE_DIR/build/crazy_square

$DRONE_DIR/build/crazy_square serial:///dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00:57600
