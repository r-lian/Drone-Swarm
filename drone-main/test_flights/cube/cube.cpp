// This code does not work yet. The main issue is the vertical velocity
// setpoints in here exceed the default limits for vertical velocity in offboard mode.
// These limits are found in params MPC_Z_VEL_MAX_DN and MPC_Z_VEL_MAX_UP
// Either increase these limits or decrease the vertical velocity setpoints.
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

void horizontal_square(mavsdk::Offboard& offboard, const std::string& offb_mode)
{
	offboard_log(offb_mode, "Fly forward 9m at 3m/s");

	Offboard::VelocityBodyYawspeed straight{};
	straight.forward_m_s = 3.0f;	//setting forward speed
	straight.right_m_s = 0.0f;	//setting right speed
	straight.down_m_s = 0.0f;	//setting down speed
	straight.yawspeed_deg_s = 0.0f; //setting yaw speed

	offboard.set_velocity_body(straight); //sends the move command with above settings
	sleep_for(seconds(3));

	offboard_log(offb_mode, "Fly left 9m at 3m/s");

	Offboard::VelocityBodyYawspeed left{};
	left.forward_m_s = 0.0f;
	left.right_m_s = -3.0f;
	left.down_m_s = 0.0f;
	left.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(left);
	sleep_for(seconds(3));

	offboard_log(offb_mode, "Fly backward 9m at 3m/s");

	Offboard::VelocityBodyYawspeed backward{};
	backward.forward_m_s = -3.0f;
	backward.right_m_s = 0.0f;
	backward.down_m_s = 0.0f;
	backward.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(backward);
	sleep_for(seconds(3));

	offboard_log(offb_mode, "Fly right 9m at 3m/s");

	Offboard::VelocityBodyYawspeed right{};
	right.forward_m_s = 0.0f;
	right.right_m_s = 3.0f;
	right.down_m_s = 0.0f;
	right.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(right);
	sleep_for(seconds(3));
}

void vertical_square(mavsdk::Offboard& offboard, const std::string& offb_mode)
{
	offboard_log(offb_mode, "Fly up 9m at 3m/s");

	Offboard::VelocityBodyYawspeed up{};
	up.forward_m_s = 0.0f;
	up.right_m_s = 0.0f;
	up.down_m_s = -3.0f;
	up.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(up);
	sleep_for(seconds(3));

	offboard_log(offb_mode, "Fly forward 9m at 3m/s");
	
	Offboard::VelocityBodyYawspeed straight{};
	straight.forward_m_s = 3.0f;		
	straight.right_m_s = 0.0f;
	straight.down_m_s = 0.0f;
	straight.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(straight);
	sleep_for(seconds(3));

	offboard_log(offb_mode, "Fly down 9m at 3m/s");

	Offboard::VelocityBodyYawspeed down{};
	down.forward_m_s = 0.0f;
	down.right_m_s = 0.0f;
	down.down_m_s = 3.0f;
	down.yawspeed_deg_s = 0.0f;
	
	offboard.set_velocity_body(down);
	sleep_for(seconds(3));

	offboard_log(offb_mode, "Fly backward 9m at 3m/s");

	Offboard::VelocityBodyYawspeed backward{};
	backward.forward_m_s = -3.0f;
	backward.right_m_s = 0.0f;
	backward.down_m_s = 0.0f;
	backward.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(backward);
	sleep_for(seconds(3));
}

bool offb_ctrl_body(mavsdk::Offboard& offboard)
{
	const std::string offb_mode = "BODY";

	// Send it once before starting offboard, otherwise it will be rejected.
	Offboard::VelocityBodyYawspeed stay{};
	offboard.set_velocity_body(stay);

	Offboard::Result offboard_result = offboard.start();
	offboard_error_exit(offboard_result, "Offboard start failed: ");
	offboard_log(offb_mode, "Offboard started");
	
	horizontal_square(offboard, offb_mode); //cube bottom
	vertical_square(offboard, offb_mode); //cube right

	offboard_log(offb_mode, "Rotate 90 deg ccw at 30deg/s");

	Offboard::VelocityBodyYawspeed rotate_90_ccw{};
	rotate_90_ccw.forward_m_s = 0.0f;
	rotate_90_ccw.right_m_s = 0.0f;
	rotate_90_ccw.down_m_s = 0.0f;
	rotate_90_ccw.yawspeed_deg_s = -30.0f; //this should make it rotate to the left

	offboard.set_velocity_body(rotate_90_ccw);
	sleep_for(seconds(3));

	vertical_square(offboard, offb_mode); //cube front
	
	offboard_log(offb_mode, "Fly forward 9m at 3m/s");
	
	Offboard::VelocityBodyYawspeed straight{};
	straight.forward_m_s = 3.0f;		
	straight.right_m_s = 0.0f;
	straight.down_m_s = 0.0f;
	straight.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(straight);
	sleep_for(seconds(3));

	offboard_log(offb_mode, "Fly right 9m at 3m/s");

	Offboard::VelocityBodyYawspeed right{};
	right.forward_m_s = 0.0f;
	right.right_m_s = 3.0f;
	right.down_m_s = 0.0f;
	right.yawspeed_deg_s = 0.0f;

	offboard.set_velocity_body(right);
	sleep_for(seconds(3));
	
	offboard_log(offb_mode, "Rotate 90 deg ccw at 30deg/s");
	offboard.set_velocity_body(rotate_90_ccw);
	sleep_for(seconds(3));

	vertical_square(offboard, offb_mode); //cube left

	offboard_log(offb_mode, "Rotate 90 deg ccw at 30deg/s");
	offboard.set_velocity_body(rotate_90_ccw);
	sleep_for(seconds(3));

	vertical_square(offboard, offb_mode); //cube back

	//top part of cube is implicit ;)
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

Telemetry::LandedStateCallback
landed_state_callback(Telemetry& telemetry, std::promise<void>& landed_promise)
{
	return [&landed_promise, &telemetry](Telemetry::LandedState landed) {
		switch (landed) {
			case Telemetry::LandedState::OnGround:
				std::cout << "On ground" << std::endl;
				break;
			case Telemetry::LandedState::TakingOff:
				std::cout << "Taking off..." << std::endl;
				break;
			case Telemetry::LandedState::Landing:
				std::cout << "Landing..." << std::endl;
				break;
			case Telemetry::LandedState::InAir:
				std::cout << "Taking off has finished." << std::endl;
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

	std::cout << "Declaring offboard promise/future" << std::endl;
	std::promise<void> offboard_promise ;
	std::future<void> offboard_future = offboard_promise.get_future();

	Telemetry::FlightMode old_flight_mode = Telemetry::FlightMode::Unknown ;

	telemetry.subscribe_flight_mode([&old_flight_mode, &offboard_promise](Telemetry::FlightMode new_flight_mode) {
		if (new_flight_mode == Telemetry::FlightMode::Offboard && new_flight_mode != old_flight_mode) {
			std::cout << "Setting offboard mode promise" << std::endl ;
			offboard_promise.set_value();
		} 

		if (old_flight_mode != new_flight_mode) {
			std::cout << "new_flight_mode = " << new_flight_mode ;
			old_flight_mode = new_flight_mode ;
		}

	} ) ;

	while (offboard_future.wait_for(milliseconds(20)) == std::future_status::timeout ) {
		std::cout << "Setting initial offboard setpoint." << std::endl;
		Offboard::VelocityBodyYawspeed stay{};
		offboard.set_velocity_body(stay);
	}

	bool ret = false ;

	//  using body co-ordinates
	ret = offb_ctrl_body(offboard);
	if (ret == false) {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
