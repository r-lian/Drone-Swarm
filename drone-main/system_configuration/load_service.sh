#!/bin/sh

sudo systemctl enable --now $DRONE_SYS_CONFIG_DIR/drone_programs.service
