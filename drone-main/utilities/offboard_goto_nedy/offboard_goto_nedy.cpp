/* David Castellon */
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
		flag_args["--north"].empty()              ||
		flag_args["--east"].empty()             ||
		flag_args["--down"].empty()              ||
		flag_args["--yaw"].empty()                   ||
		flag_args["--speed"].empty()
	) {
		cout << parser.get_command() << " "
			<< connection_url_flag << " "
			<< north_flag << " "
			<< east_flag << " "
			<< down_flag << " "
			<< yaw_flag << " "
			<< speed_flag << "\n"
			<< connection_url_format;

		return -1;
	}

	// 86400 seconds in one day.
	if ( !drone.wait_for_offboard_mode(86400) ) return 1;
	// if ( !drone.initialize_offboard_mode(86400) ) return 1;

	drone.action_goto_nedy(
		stof(flag_args["--north"]),
		stof(flag_args["--east"]),
		stof(flag_args["--down"]),
		stof(flag_args["--yaw"]),
		stof(flag_args["--speed"])
	);

	return 0;
}
