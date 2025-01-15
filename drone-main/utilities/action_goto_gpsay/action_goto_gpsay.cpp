/* David Castellon */
#include "helpers.h"

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
	double latitude = stod(flag_args["--latitude"]);
	double longitude = stod(flag_args["--longitude"]);
	double altitude = stod(flag_args["--altitude"]);
	float yaw = stof(flag_args["--yaw"]);
	float speed = stof(flag_args["--speed"]);

	drone.action_goto_gpsay(
		latitude,
		longitude,
		altitude,
		yaw,
		speed
	);

	return 0;
}
