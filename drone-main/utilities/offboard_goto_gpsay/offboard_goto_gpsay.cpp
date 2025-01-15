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

	// 86400 seconds in one day.
	if ( !drone.wait_for_offboard_mode(86400) ) return 1;
	// if ( !drone.initialize_offboard_mode(86400) ) return 1;
	
	drone.offboard_goto_gpsay(
		stod(flag_args["--latitude"]),
		stod(flag_args["--longitude"]),
		stod(flag_args["--altitude"]),
		stof(flag_args["--yaw"]),
		stof(flag_args["--speed"])
	);
	return 0;
}
