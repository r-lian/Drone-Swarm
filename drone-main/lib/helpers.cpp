#include <iostream>
#include <string>
#include <chrono>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <command_line_argument_parser/command_line_argument_parser.h>
#include <mutex/mutex.h>
#include <drone/drone.h>

using namespace std;
using namespace mavsdk;
// #include "helpers.h"

void print_float_array(float float_array[], int size) {
	for (int i = 0; i < size; i++) {
		printf("float_array[%i] = %f\n", i, float_array[i]);
	}
}

void print_double_array(double double_array[], int size) {
	for (int i = 0; i < size; i++) {
		cout << "double_array[" << i << "] = " << double_array[i] << "\n";
	}
}

// bool print_usage_if_could_not_connect(Drone &drone, Command_Line_Argument_Parser parser) {
// 	ConnectionResult connection_result = drone.get_connection_result();
// 	bool is_connected_to_flight_controller = drone.is_connected_to_flight_controller();
// 	if (
// 		( connection_result != ConnectionResult::Success ) ||
// 		( !is_connected_to_flight_controller )
// 	) {
// 		cout << parser.get_command() << " --connection_url " << "<connection_url>" << "\n";
// 		cout << "<connection_url> format should be :\n"
// 			<< "For TCP : tcp://[server_host][:server_port]\n"
// 			<< "For UDP : udp://[bind_host][:bind_port]\n"
// 			<< "For Serial : serial:///path/to/serial/dev[:baudrate]\n"
// 			<< "For example, to connect to the simulator use URL: udp://:14540\n" ;
// 		return false;
// 	}
// 	return true;
// }

bool wait_until_in_air(Drone &drone, chrono::seconds timeout) {
	promise<void> prom;
	future<void> fut = prom.get_future();

	drone.telemetry->set_rate_in_air(drone.get_callback_frequency_in_hz());
	drone.telemetry->subscribe_in_air( [&](bool _in_air){
		drone.in_air = _in_air;
		if ( drone.in_air == false ) return;
		cout << "drone.in_air = true\n" << endl;
		// cout << seperator << endl;
		drone.telemetry->subscribe_in_air(nullptr);
		prom.set_value();
	});
	if ( fut.wait_for(timeout) == future_status::timeout ) return false;
	return true;
}

// bool print_usage_if_no_connection_url(Command_Line_Argument_Parser parser){
// 	map<string, string> flag_args = parser.get_flag_arguments();
// 	bool connection_url_is_empty = flag_args["--connection_url"].empty();
// 	if ( !connection_url_is_empty ) {
// 		cout << parser.get_command() << " --connection_url " << "<connection_url>" << "\n";
// 		cout << "<connection_url> format should be :\n"
// 			<< "For TCP : tcp://[server_host][:server_port]\n"
// 			<< "For UDP : udp://[bind_host][:bind_port]\n"
// 			<< "For Serial : serial:///path/to/serial/dev[:baudrate]\n"
// 			<< "For example, to connect to the simulator use URL: udp://:14540\n" ;
// 		return false;
// 	}
// 	return true;
// }

// int main (int argc, char *argv[]) {
// 	cout << "Parsing command line arguments." << endl;
// 
// 	Drone_Program_Command_Line_Arguments drone_command_line_arguments(argc, argv);
// 
// 	cout << "Checking for basic command line arguments." << endl;
// 	bool has_basic_arguments = drone_command_line_arguments.has_basic_arguments();
// 	if ( !has_basic_arguments ) {
// 		cout << drone_command_line_arguments.get_usage() << endl;
// 		return 0;
// 	}
// 
// 	cout << "Retrieving connection url from command line arguments." << endl;
// 	string connection_url = drone_command_line_arguments.get_connection_url();
// 
// 	cout << "Constructing drone object." << endl;
// 	Drone drone(connection_url);
// 
// 	cout << "Checking if flight controller is connected." << endl;
// 	if ( !drone.get_is_connected_to_flight_controller() ) {
// 		cout << "Could not connect to flight controller!" << endl;
// 		return 1;
// 	}
// }
