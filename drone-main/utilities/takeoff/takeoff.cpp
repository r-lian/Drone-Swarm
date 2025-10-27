#include "helpers.h"

int main(int argc, char** argv)
{
	// Handles Command Line Arguments
	Command_Line_Argument_Parser parser(argc, argv);
	map<string, string> flag_args = parser.get_flag_arguments();

	// If no connection url tries to connect with empty string.
	Drone drone(flag_args["--connection_url"]);

	if (
		(!drone.is_connected_to_flight_controller())
	) {
		cout << parser.get_command() << " "
			<< connection_url_flag << "\n"
			<< connection_url_format;

		return -1;
	}
	drone.action->arm();
	if (drone.takeoff()) return 0;

	return -1;
}
