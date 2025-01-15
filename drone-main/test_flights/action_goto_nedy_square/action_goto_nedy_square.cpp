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
		flag_args["--side_length"].empty() ||
		flag_args["--speed"].empty()
	) {
		cout << parser.get_command() << " "
			<< connection_url_flag << " "
			<< side_length_flag << " "
			<< speed_flag << "\n"
			<< connection_url_format;

		return -1;
	}

	float speed = stof(flag_args["--speed"]);
	float side_length = stof(flag_args["--side_length"]);

	// square action nedy
	drone.action_goto_nedy(0.0f, 0.0f, -1 * side_length, 0.0f, speed);
	drone.action_goto_nedy(side_length, 0.0f, -1 * side_length, 0.0f, speed);
	drone.action_goto_nedy(side_length, side_length, -1 * side_length, 0.0f, speed);
	drone.action_goto_nedy(0.0f, side_length, -1 * side_length, 0.0f, speed);
	drone.action_goto_nedy(0.0f, 0.0f, -1 * side_length, 0.0f, speed);

	return 0;
}
