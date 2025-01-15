/**
 * @file offboard_velocity.cpp
 * @brief Example that demonstrates offboard velocity control in local NED and
 * body coordinates
 *
 * @authors Author: Julian Oes <julian@oes.ch>,
 *                  Shakthi Prashanth <shakthi.prashanth.m@intel.com>
 */

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string& message)
{
	if (result != Action::Result::Success) {
		std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
		exit(EXIT_FAILURE);
	}
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string& message)
{
	if (result != Offboard::Result::Success) {
		std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
		exit(EXIT_FAILURE);
	}
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string& message)
{
	if (result != ConnectionResult::Success) {
		std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
		exit(EXIT_FAILURE);
	}
}

// Logs during Offboard control
inline void offboard_log(const std::string& offb_mode, const std::string msg)
{
	std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using body co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log
 * otherwise.
 */
bool offb_ctrl_body(mavsdk::Offboard& offboard)
{
	const std::string offb_mode = "BODY";

	// Send it once before starting offboard, otherwise it will be rejected.
	Offboard::VelocityBodyYawspeed stay{};
	offboard.set_velocity_body(stay);

	Offboard::Result offboard_result = offboard.start();
	offboard_error_exit(offboard_result, "Offboard start failed: ");
	offboard_log(offb_mode, "Offboard started");

	offboard_log(offb_mode, "Fly forward 1m at .1m/s");

	Offboard::VelocityBodyYawspeed straight{};
	straight.forward_m_s = 0.1f;
	straight.right_m_s = 0.0f;
	straight.down_m_s = 0.0f;
	straight.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(straight);
	sleep_for(seconds(10));

	offboard_log(offb_mode, "Fly backward 1m at .1m/s");

	Offboard::VelocityBodyYawspeed backward{};
	backward.forward_m_s = -0.1f;
	backward.right_m_s = 0.0f;
	backward.down_m_s = 0.0f;
	backward.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(backward);
	sleep_for(seconds(10));

	offboard_result = offboard.stop();
	offboard_error_exit(offboard_result, "Offboard stop failed: ");
	offboard_log(offb_mode, "Offboard stopped");

	return true;
}

void wait_until_discover(Mavsdk& mavsdk)
{
	std::cout << "Waiting to discover system..." << std::endl;
	std::promise<void> discover_promise;
	auto discover_future = discover_promise.get_future();

	mavsdk.subscribe_on_new_system([&mavsdk, &discover_promise]() {
			const auto system = mavsdk.systems().at(0);

			if (system->is_connected()) {
			std::cout << "Discovered system" << std::endl;
			discover_promise.set_value();
			}
			});

	discover_future.wait();
}

void usage(std::string bin_name)
{
	std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
		<< "Connection URL format should be :" << std::endl
		<< " For TCP : tcp://[server_host][:server_port]" << std::endl
		<< " For UDP : udp://[bind_host][:bind_port]" << std::endl
		<< " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
		<< "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

Telemetry::LandedStateCallback landed_state_callback(Telemetry& telemetry, std::promise<void>& landed_promise)
{
	return [&landed_promise, &telemetry](Telemetry::LandedState landed) {
		switch (landed) {
			case Telemetry::LandedState::OnGround:
				std::cout << "On ground." << std::endl;
				break;
			case Telemetry::LandedState::TakingOff:
				std::cout << "Taking off."<< std::endl;
				break;
			case Telemetry::LandedState::Landing:
				std::cout << "Landing." << std::endl;
				break;
			case Telemetry::LandedState::InAir:
				std::cout << "In Air." << std::endl;
				telemetry.subscribe_landed_state(nullptr);
				landed_promise.set_value();
				break;
			case Telemetry::LandedState::Unknown:
				std::cout << "Unknown landed state." << std::endl;
				break;
		}
	};
}

int main(int argc, char** argv)
{
	Mavsdk mavsdk;
	std::string connection_url;
	ConnectionResult connection_result;

	if (argc == 2) {
		connection_url = argv[1];
		connection_result = mavsdk.add_any_connection(connection_url);
	} else {
		usage(argv[0]);
		return 1;
	}

	if (connection_result != ConnectionResult::Success) {
		std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
			<< NORMAL_CONSOLE_TEXT << std::endl;
		sleep_for(seconds(5));
		return 1;
	}

	// Wait for the system to connect via heartbeat
	wait_until_discover(mavsdk);

	// System got discovered.
	auto system = mavsdk.systems().at(0);
	auto action = Action{system};
	auto offboard = Offboard{system};
	auto telemetry = Telemetry{system};

	while (!telemetry.health_all_ok()) {
		std::cout << "Waiting for system to be healthy." << std::endl;
	}
	std::cout << "System is healthy." << std::endl;

	std::promise<void> in_air_promise;
	auto in_air_future = in_air_promise.get_future();

	telemetry.subscribe_landed_state(landed_state_callback(telemetry, in_air_promise));

	// Send it once before starting offboard, otherwise it will be rejected.
	Offboard::VelocityBodyYawspeed stay{};
	offboard.set_velocity_body(stay);

	std::promise<void> offboard_promise ;
	std::future<void> offboard_future = offboard_promise.get_future();

	Telemetry::FlightMode old_flight_mode = Telemetry::FlightMode::Unknown ;
	telemetry.subscribe_flight_mode([&old_flight_mode, &offboard_promise](Telemetry::FlightMode new_flight_mode) {
		if (old_flight_mode != new_flight_mode) {
			std::cout << "new_flight_mode = " << new_flight_mode ;
			old_flight_mode = new_flight_mode ;
		}

		if (new_flight_mode == Telemetry::FlightMode::Offboard) {
			offboard_promise.set_value();
		} 
	} ) ;

	offboard_future.wait();
	in_air_future.wait();

	bool ret = false ;

	//  using body co-ordinates
	ret = offb_ctrl_body(offboard);
	if (ret == false) {
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
