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
	float north = stof(flag_args["--north"]);
	float east = stof(flag_args["--east"]);
	float down = stof(flag_args["--down"]);
	float yaw = stof(flag_args["--yaw"]);
	float speed = stof(flag_args["--speed"]);

	cout << "heading to nedy:\n"
		<< "north: " << north << "\n"
		<< "east: " << east << "\n"
		<< "down: " << down << "\n"
		<< seperator << "\n";

	// drone.action_goto_nedy(
	// 	north,
	// 	east,
	// 	down,
	// 	yaw,
	// 	speed
	// );

	// works
	std::tuple<double, double, float> gpsa = drone.ned_to_gpsa(
		north,
		east,
		down
	);

	// drone.action_goto_gpsay(
	// 	std::get<0>(gpsa),
	// 	std::get<1>(gpsa),
	// 	std::get<2>(gpsa),
	// 	yaw,
	// 	speed
	// );

	// does not work
	std::tuple<float, float, float> ned = drone.gpsa_to_ned(
		std::get<0>(gpsa),
		std::get<1>(gpsa),
		std::get<2>(gpsa)
	);
	cout << "heading towards:\n"
		<< "north: " << std::get<0>(ned) << "\n"
		<< "east: " << std::get<1>(ned) << "\n"
		<< "down: " << std::get<2>(ned) << "\n";

	sleep_for(std::chrono::seconds(10));

	drone.action_goto_nedy(
		std::get<0>(ned),
		std::get<1>(ned),
		std::get<2>(ned),
		yaw,
		speed
	);

	// gpsa = drone.gpsa_to_ned(
	// 	std::get<0>(ned),
	// 	std::get<1>(ned),
	// 	std::get<2>(ned)
	// );

	// // // cout << seperator << "\n"
	// // // 	<< "Calculated ned:\n"
	// // // 	<< "north: " << std::get<0>(ned) << "\n"
	// // // 	<< "east: " << std::get<1>(ned) << "\n"
	// // // 	<< "down: " << std::get<2>(ned) << "\n"
	// // // 	<< seperator << "\n";


	return 0;
}
