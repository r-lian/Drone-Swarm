# RPI Setup

## Hardware Overview

Our drones have a rpi, pixhawk, escs, motors, batteries, and sensors.
The rpi sends abstract commands to the pixhawk to execute; for example,
go this location or go this fast in this direction.

The purpose of this project is to implement different tasks for sets of
drones to perform using the batman network protocol for drone
communication.

## Pixhawk Setup

Write the parameter file onto the pixhawk.

## Rpi Setup

### Method 1: Fresh Arch Arm Install 

Get a micro sd card with at least 8GB, then flash archlinux arm
onto the sd card by running the
system\_configuration/install\_arch\_arm.sh script.

	./system_configuration/install_arch_arm.sh /dev/sdb

Copy the drone project onto a flash drive. Insert the
sd card and the flash drive into the rpi,
turn on the rpi, connect to ethernet, and create
a user with a password.

	./system_configuration/create_user.sh user passwd

Afterwards logout of root user and login to the newly created user.
Then, install the required packages.

	./system_configuration/install_required_packages.sh

Then load batman\_adv

	./system_configuration/load_batman_adv.sh

### Method 2: Formatting then Copying the Files

Format the rpi.

Copy all the files from the boot partition and the root
partition into the sd card's boot and root partition.


Delete all the db files in /var/lib/pacman/sync/.

	rm /var/lib/pacman/sync/*

Update the package database and upgrade all the packages.

	pacman -Syu

The drone's rpi should be fully setup at this point.

### Method 3: Mirroring Already Setup SD Card

Create an iso from a fully configured 8GB sd card.

Then, flash another 8GB with the iso.

Then, delete all the db files in /var/lib/pacman/sync/.

	rm /var/lib/pacman/sync/*

Then, update the package database and upgrade all the packages.

	pacman -Syu

The drone's rpi should be fully setup at this point.

## Project Structure

	/drone
		/system_configuration: Contains rpi scripts and the pixhawk's parameter's file.
		/lib: Contains helper functions and classes.
		/utilities: Contains programs for querying the drone for information.
			/telemetry: Reads pixhawk telemetry. (uses mavsdk)
		/test_flights: Contains test flight programs
			velocity_timeout_square: Fly in a square using velocity and timeouts.
			goto_square: Fly in a square using goto functions that check the current drone's current position.
			crazy-square: Fly in a square using velocity and position readings.(max speed may be dangerous)
			gps-square: Fly in a square using gps setpoints and position readings.
			partner-leader: Fly 
