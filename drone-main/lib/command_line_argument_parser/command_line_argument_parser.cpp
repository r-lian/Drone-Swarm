#include "command_line_argument_parser.h"

//////////////////////////////////////////////////
// Command_Line_Argument_Parser
//////////////////////////////////////////////////
Command_Line_Argument_Parser::Command_Line_Argument_Parser(int argc, char *argv[]) {
	int next_arg_index = 0;
	char *arg = NULL;

	command = argv[0];
	for (
		int arg_index = 1;
		arg_index < argc;
		arg_index++
	) {
		next_arg_index = arg_index + 1;

		if ( argv[arg_index][0] == '-') {

			if ( argv[arg_index][1] == '\0' ) {
				flag_arguments.insert( pair<string, string>(
					argv[arg_index],
					string()
				) );
			} else if ( next_arg_index < argc ) {
				arg = argv[next_arg_index];
				flag_arguments.insert( pair<string, string>(
					argv[arg_index],
					argv[next_arg_index]
				) );
				arg_index++;
			}
		} else {
			lone_arguments.push_back(argv[arg_index]);
		}
	}
}

string Command_Line_Argument_Parser::get_command() { return command; }
vector<string> Command_Line_Argument_Parser::get_lone_arguments() { return lone_arguments; }
map<string, string> Command_Line_Argument_Parser::get_flag_arguments() { return flag_arguments; }
// string Command_Line_Argument_Parser::get_usage() { return usage; }
// void Command_Line_Argument_Parser::append_to_usage(string new_usage_info) { usage += new_usage_info; }
// string Command_Line_Argument_Parser::get_details() { return details; }
// void Command_Line_Argument_Parser::append_to_details(string new_detail_info) { usage += new_detail_info; }

//////////////////////////////////////////////////
// Drone_Program_Command_Line_Argument_Parser
//////////////////////////////////////////////////
// Drone_Program_Command_Line_Argument_Parser::Drone_Program_Command_Line_Argument_Parser(
// 	int argc,
// 	char *argv[]
// ) : Command_Line_Argument_Parser(argc, argv) {
// 
// 	usage = "Usage: " + command + " " + "<connection_url>";
// }
// 
// string Drone_Program_Command_Line_Argument_Parser::get_connection_url() {
// 	return ( ( 0 < lone_arguments.size() ) ? lone_arguments[0] : "" ) ;
// }
// 
// bool Drone_Program_Command_Line_Argument_Parser::has_basic_arguments() {
// 	if ( get_connection_url() == "" ) {
// 		return false;
// 	}
// 	return true;
// }

// #include <iostream>
// int main (int argc, char *argv[]) {
// 	Drone_Program_Command_Line_Argument_Parser drone_program_command_line_arguments(argc, argv);
// 	std::string connection_url = drone_program_command_line_arguments.get_connection_url();
// 	if ( connection_url == "" )
// 		std::cout << drone_program_command_line_arguments.get_usage() << std::endl;
// }
