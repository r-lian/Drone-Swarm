#ifndef COMMAND_LINE_ARGUMENT_PARSER_H

#define COMMAND_LINE_ARGUMENT_PARSER_H

#include <vector>
#include <map>
#include <iostream>
#include <string>

using namespace std;

class Command_Line_Argument_Parser {
protected:
	string command;
	string usage;
	// string details;

	vector<string> lone_arguments;
	map<string, string> flag_arguments;
public:
	Command_Line_Argument_Parser(int argc, char *argv[]) ;

	vector<string> get_lone_arguments();
	map<string, string> get_flag_arguments();

	string get_command();
	// string get_usage();
	// void append_to_usage(string new_usage_info);
	// string get_details();
	// void append_to_details(string new_detail_info);
};

// class Drone_Program_Command_Line_Arguments : public Command_Line_Arguments {
// public:
// 	Drone_Program_Command_Line_Arguments(int argc, char *argv[]);
// 
// 	string get_connection_url();
// 	bool has_basic_arguments();
// };

#endif
