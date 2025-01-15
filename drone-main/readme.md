# Drone

The purpose of this project is to implement drone swarm
shape drawing using drone swarm data sharing.

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

Note: if the droneswarm sharing idea is done in its own threads, I could make
the droneswarm dataset essentially a global variable that can be queried for information
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
