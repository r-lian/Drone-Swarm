export DRONE_DIR=$PWD
export DRONE_SYS_CONFIG_DIR=$PWD/system_configuration
echo "export DRONE_DIR=$PWD" >> ~/.bashrc
echo "export DRONE_SYS_CONFIG_DIF=$DRONE_SYS_CONFIG_DIR" >> ~/.bashrc
echo "Environment=\"DRONE_DIR=$DRONE_DIR\"" >> $DRONE_SYS_CONFIG_DIR/drone_programs.service
