# Drone

The purpose of this project is to implement drone swarm
shape drawing using drone swarm data sharing.

## Collaborators: David Castellon, Graduate Research Assistant at the University of Louisville

## Current Idea for drone swarm data sharing:

Each drone has a drone swarm data set that they keep track of.
The drone swarm data set contains a collection of drone data
sets with timestamps. (Every drone has its own timestamp)
The drone data set can contain sensor information. For the first
iteration, I'm only going to use gps data.

The updating works as follows:

Drone x updates its timestamp in its own drone data set, then drone x broadcasts its
drone swarm data set to the set of drones y drone x is directly connected to in the
mesh network. Drone z is a member of drone set y.

When drone z receives a drone swarm dataset from drone x, drone z
then updates its own drone swarm dataset with the received drone swarm dataset
using the timestamps to indicate which drone data sets to update.

Note: if the drone swarm sharing idea is done in its own threads, I could make
the drone swarm dataset essentially a global variable that can be queried for information
instead of the current leader is sending gps location to each ip address in seperate threads.

## Pixhawk Setup

Write the parameter file onto the pixhawk.

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

# Developer Setup

## Linux Setup

1. Install gazebo.
2. Install qgroundcontrol.
3. Install mavsdk.
4. Clone PX4-Autopilot source code.
5. Clone this repo.

## Building Gazebo and QGroundControl from source:

For officially supported linux distributions you should not need to install gazebo or qgroundcontrol
from source. Just install the package from the repo or follow the official instructions.

If you are building gazebo from source, you may need to install specific versions of libraries. For
archlinux you need to install ignition-msg6 from source. ignition-msg6 is not in the aur currently,
but it's easy to build and install from source. (Making the aur package is on my to-do list .)

## Gazebo Simulator Setup

Build and install the gazebo dependencies not in the aur (ignition-msg6). Then install gazebo-git.

	make ignition-msg6
	PX4-Autopilot/Tools/gazebo_sitl_multiple_run.sh -s "iris:3,plane:2"

## Drone Control System

A comprehensive drone control and navigation system built using MAVSDK, implementing advanced control modes and perception capabilities.

## Project Structure

### Core Library (`lib/`)
- **Drone Class** (`drone.h`, `drone.cpp`)
  - Core class managing all drone operations
  - Implements connection management, state tracking, and control systems
  - Supports multiple control modes and coordinate frames
  - Handles telemetry and sensor data processing

- **Helper Utilities** (`helpers.h`, `helpers.cpp`)
  - Common utility functions
  - Math operations for coordinate transformations
  - Data validation and error checking

- **System Components**
  - Command Line Parser: Processes command-line arguments
  - Mutex Implementation: Thread-safe operations
  - Thread Tracker: Manages concurrent operations
  - Timer: Precise timing control
  - UDP Socket: Network communication for multi-drone setups

### Test Flights (`test_flights/`)
- **Position Control Tests**
  - `action_goto_gpsay_square/`: GPS-based square trajectory
  - `action_goto_nedy_square/`: NED-frame square pattern
  - `offboard_goto_gpsay_square/`: GPS waypoint navigation
  - `offboard_goto_nedy_square/`: NED waypoint control
  - `offboard_set_position_square/`: Direct position control tests

- **Velocity Control Tests**
  - `velocity_timeout_square/`: Velocity-based movement with timeout handling
  - `crazy_square/`: Complex velocity-based patterns

- **Advanced Flight Patterns**
  - `cube/`: 3D cube trajectory implementation
  - `leader/`: Leader drone implementation for swarm control
  - `partner/`: Follower drone implementation

### Utilities (`utilities/`)
- **Navigation Tools**
  - `action_goto_gpsay/`: GPS-based navigation utilities
  - `action_goto_nedy/`: NED-frame navigation
  - `offboard_goto_gpsay/`: Offboard GPS control
  - `offboard_goto_nedy/`: Offboard NED control
  - `takeoff/`: Takeoff procedures
  - `telemetry/`: Telemetry monitoring tools

## Control Architecture

### Flight Control System
- Utilizes MAVSDK for drone control and communication
- Implements multiple control modes:
  - Offboard Control: Direct velocity and position control
  - Body Frame Control: Using VelocityBodyYawspeed for precise movement
  - NED (North-East-Down) Frame Control: For global positioning

### Control Modes
- **Velocity Control**:
  - Implements both body-frame and NED-frame velocity control
  - Body frame control parameters:
    - `forward_m_s`: Forward velocity
    - `right_m_s`: Lateral velocity
    - `down_m_s`: Vertical velocity
    - `yawspeed_deg_s`: Yaw rotation rate
  - Allows precise movement control with configurable speeds

- **Position Control**:
  - Uses GPS and local position estimation
  - Supports both relative and absolute positioning
  - Implements waypoint navigation through action and offboard commands

## Perception System

### Sensor Integration
- Integrates multiple sensors:
  - IMU (Inertial Measurement Unit)
  - GPS
  - Distance sensors
  - Attitude sensors
  - Camera attitude tracking
  - Odometry systems

### State Estimation
- Implements comprehensive state estimation through telemetry subscriptions:
  - Position tracking (3D coordinates)
  - Velocity estimation (NED frame)
  - Attitude estimation (orientation)
  - Height above ground
  - GPS information
  - Battery status
  - Landing state detection

## Control and Perception Integration

### Feedback Control
- Implements a closed-loop control system where sensor data feeds back into control decisions
- Uses telemetry data at configurable rates (set through `set_rate_*` functions)
- Key feedback loops:
  - Position feedback for waypoint navigation
  - Attitude feedback for stability
  - Velocity feedback for movement control

### State Machine
- Implements various flight states:
  - Takeoff
  - Landing
  - In-air detection
  - Offboard control
  - Emergency procedures

## System Architecture

### Communication
- Uses MAVSDK for standardized communication
- Supports multiple connection types:
  - TCP
  - UDP
  - Serial
- Implements robust error handling and connection management

### Data Flow
- **Sensor Data**:
  - High-frequency telemetry updates
  - Configurable sampling rates for different sensors
  - Efficient data handling through callback functions
- **Control Commands**:
  - Asynchronous command execution
  - Error checking and validation
  - Safety-first approach with multiple failsafes

## Safety Features

### Error Handling
- Comprehensive error checking for all control operations
- Graceful failure handling
- Emergency procedures implementation

### State Validation
- Continuous monitoring of system state
- Pre-flight checks
- Connection status monitoring

## Key Technical Features

### Control Techniques
- **Offboard Control**:
  - Direct control through velocity and position commands
  - Safety checks and error handling
  - Both synchronous and asynchronous command execution
- **Precision Movement**:
  - Square trajectory implementation with precise timing
  - Velocity-based movement with configurable speeds
  - Support for both relative and absolute positioning

### Perception Techniques
- **Sensor Fusion**:
  - Integrates multiple sensor inputs for robust state estimation
  - Implements odometry for position tracking
  - Uses IMU data for attitude estimation
- **State Monitoring**:
  - Real-time telemetry monitoring at configurable frequencies
  - Comprehensive state tracking

## System Requirements and Setup
- See `simulator_setup.md` for simulation environment setup
- See `rpi_setup.md` for Raspberry Pi deployment instructions
- System configuration files available in `system_configuration/`

## Build System
- CMake-based build system
- Modular architecture with separate libraries and executables
- Comprehensive test suite in `test_flights/` and `test_scripts/`
