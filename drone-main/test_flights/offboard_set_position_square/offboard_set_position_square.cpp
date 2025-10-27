#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <utility>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "helpers.h"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

int main(int argc, char** argv)
{
	// Handles Command Line Arguments
	Command_Line_Argument_Parser parser(argc, argv);
	map<string, string> flag_args = parser.get_flag_arguments();

	// If no connection url tries to connect with empty string.
	Drone drone(flag_args["--connection_url"]);

	if (
		(!drone.is_connected_to_flight_controller()) ||
		flag_args["--latitude"].empty()              ||
		flag_args["--longitude"].empty()             ||
		flag_args["--altitude"].empty()              ||
		flag_args["--yaw"].empty()                   ||
		flag_args["--speed"].empty()
	) {
		cout << parser.get_command() << " "
			<< connection_url_flag << " "
			<< latitude_flag << " "
			<< longitude_flag << " "
			<< altitude_flag << " "
			<< yaw_flag << " "
			<< speed_flag << "\n"
			<< connection_url_format;

		return -1;
	}

	// 86400 seconds in one day.
	if ( !drone.wait_for_offboard_mode(86400) ) return 1;
	// if ( !drone.initialize_offboard_mode(86400) ) return 1;

	drone.offboard->set_position_ned({0.0f, 0.0f, -5.0f, 0.0f});
	sleep_for(seconds(5));
	drone.offboard->set_position_ned({10.0f, 10.0f, -15.0f, 90.0f});
	sleep_for(seconds(5));
	drone.offboard->set_position_ned({0.0f, 10.0f, -5.0f, 180.0f});
	sleep_for(seconds(5));
	drone.offboard->set_position_ned({10.0f, 0.0f, -15.0f, 270.0f});
	sleep_for(seconds(5));


	return 0;
}
