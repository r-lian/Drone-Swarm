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

	std::tuple<double, double, float> gps_altitude1 = drone.ned_to_gpsa(
		0.0f,
		0.0f,
		-1 * side_length
	);

	std::tuple<double, double, float> gps_altitude2 = drone.ned_to_gpsa(
		side_length,
		0.0f,
		-1 * side_length
	);

	std::tuple<double, double, float> gps_altitude3 = drone.ned_to_gpsa(
		side_length,
		side_length,
		-1 * side_length
	);

	std::tuple<double, double, float> gps_altitude4 = drone.ned_to_gpsa(
		0.0f,
		side_length,
		-1 * side_length
	);

	// 86400 seconds in one day.
	if ( !drone.wait_for_offboard_mode(86400) ) return 1;
	// if ( !drone.initialize_offboard_mode(86400) ) return 1;
	

	// square offboard nedy
	drone.offboard_goto_gpsay(
		std::get<0>(gps_altitude1),
		std::get<1>(gps_altitude1),
		std::get<2>(gps_altitude1),
		0.0f,
		speed
	);

	drone.offboard_goto_gpsay(
		std::get<0>(gps_altitude2),
		std::get<1>(gps_altitude2),
		std::get<2>(gps_altitude2),
		0.0f,
		speed
	);
		
	drone.offboard_goto_gpsay(
		std::get<0>(gps_altitude3),
		std::get<1>(gps_altitude3),
		std::get<2>(gps_altitude3),
		0.0f,
		speed
	);

	drone.offboard_goto_gpsay(
	 	std::get<0>(gps_altitude4),
	 	std::get<1>(gps_altitude4),
		std::get<2>(gps_altitude4),
		0.0f,
		speed
	);

	drone.offboard_goto_gpsay(
		std::get<0>(gps_altitude1),
		std::get<1>(gps_altitude1),
		std::get<2>(gps_altitude1),
		0.0f,
		speed
	);
	return 0;
}
