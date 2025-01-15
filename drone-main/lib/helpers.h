#ifndef HELPERS_H
#include <iostream>
#include <thread>
#include <string>
#include <chrono>
#include <set>
#include <vector>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/unistd.h>
#include <sys/fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <command_line_argument_parser/command_line_argument_parser.h>
#include <mutex/mutex.h>
#include <timer/timer.h>
#include <thread_tracker/thread_tracker.h>
#include <drone/drone.h>

using namespace std;
using namespace mavsdk;

char seperator[] = "--------------------------------------------------";
char connection_url_flag[] = "--connection_url <connection_url>";
char latitude_flag[] = "--latitude <latitude>";
char longitude_flag[] = "--longitude <longitude>";
char altitude_flag[] = "--altitude <altitude>";
char north_flag[] = "--north <north_m>";
char east_flag[] = "--east <east_m>";
char down_flag[] = "--down <down_m>";
char yaw_flag[] = "--yaw <yaw_deg>";
char speed_flag[] = "--speed <speed_m_s>";
char side_length_flag[] = "--side_length <length_m>";

char connection_url_format[] = "<connection_url> format should be:\n"
	"For TCP: tcp://[server_host][:server_port]\n"
	"For UDP: udp://[bind_host][:bind_port]\n"
	"For Serial: serial:///path/to/serial/dev[:baudrate]\n"
	"For example, to connect to the simulator use URL: udp://:14540\n";

inline bool operator!=(Telemetry::Position lh, Telemetry::Position rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::EulerAngle lh, Telemetry::EulerAngle rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::Quaternion lh, Telemetry::Quaternion rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::AngularVelocityBody lh, Telemetry::AngularVelocityBody rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::VelocityNed lh, Telemetry::VelocityNed rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::GpsInfo lh, Telemetry::GpsInfo rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::Battery lh, Telemetry::Battery rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::Health lh, Telemetry::Health rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::RcStatus lh, Telemetry::RcStatus rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::StatusText lh, Telemetry::StatusText rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::ActuatorControlTarget lh, Telemetry::ActuatorControlTarget rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::ActuatorOutputStatus lh, Telemetry::ActuatorOutputStatus rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::Odometry lh, Telemetry::Odometry rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::PositionVelocityNed lh, Telemetry::PositionVelocityNed rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::GroundTruth lh, Telemetry::GroundTruth rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::FixedwingMetrics lh, Telemetry::FixedwingMetrics rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::Imu lh, Telemetry::Imu rh) { return !(lh == rh); }
inline bool operator!=(Telemetry::DistanceSensor lh, Telemetry::DistanceSensor rh) { return !(lh == rh); }
inline bool operator==(struct sockaddr_in lh, struct sockaddr_in rh) {
	return (
		(lh.sin_family == rh.sin_family) &&
		(lh.sin_addr.s_addr == rh.sin_addr.s_addr) &&
		(lh.sin_port == rh.sin_port)
	);
}
inline bool operator<(struct sockaddr_in lh, struct sockaddr_in rh) {
	if ( lh.sin_addr.s_addr == rh.sin_addr.s_addr ) {
		return ( lh.sin_port < rh.sin_port );
	}
	return ( lh.sin_addr.s_addr < rh.sin_addr.s_addr );
}

inline ostream& operator<<(ostream &lh, struct sockaddr_in rh) {
	return lh << rh.sin_addr.s_addr << ":" << rh.sin_port;
}

void print_float_array(float float_array[], int size);
void print_double_array(double double_array[], int size);
void drone_program_usage(Drone &drone, string usage);
bool wait_until_in_air(chrono::seconds timeout);

// bool print_usage_if_no_connection_url(Command_Line_Argument_Parser parser);
// bool print_usage_if_could_not_connect(Drone &drone, Command_Line_Argument_Parser parser);

#endif
