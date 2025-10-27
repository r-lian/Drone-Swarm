#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/types.h>

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

// #include <mavsdk/mavsdk.h>
// #include <mavsdk/plugins/action/action.h>
// #include <mavsdk/plugins/offboard/offboard.h>
// #include <mavsdk/plugins/telemetry/telemetry.h>
// 
#include "helpers.h"
// using namespace mavsdk;
using namespace std;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define PORT     8080
#define C_PORT     8081
#define MAXLINE 1024
#define BUFFER_SIZE    4

int main(int argc, char *argv[]) {
	cout.precision(17);
	// Handles Command Line Arguments
	Command_Line_Argument_Parser parser(argc, argv);
	map<string, string> flag_args = parser.get_flag_arguments();

	// If no connection url tries to connect with empty string.
	Drone drone(flag_args["--connection_url"]);
	drone.initialize_offboard_mode(10);
	if (
		(!drone.is_connected_to_flight_controller()) ||
		flag_args["--north"].empty() ||
		flag_args["--east"].empty() ||
		flag_args["--down"].empty() ||
		flag_args["--yaw"].empty()
	) {
		cout << parser.get_command() << " "
			<< connection_url_flag << " "
			<< north_flag << " "
			<< east_flag << " "
			<< down_flag << " "
			<< yaw_flag << "\n"
			<< connection_url_format << "";

		return -1;
	}


	double receive_buffer[BUFFER_SIZE] = {0.0, 0.0, 0.0, 0.0 };
	double send_buffer[BUFFER_SIZE] = {0.0, 0.0, 0.0, 0.0 };

	int sockfd;

	struct sockaddr_in servaddr, cliaddr;
	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));


	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}

	// Bind the socket with the server address
	if ( bind(
		sockfd,
		(const struct sockaddr *)&servaddr, 
		sizeof(servaddr)
	) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	// Filling server information
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
	servaddr.sin_addr.s_addr = INADDR_ANY;

	int n;
	socklen_t len;

	// create the send thread
	struct thread_tracker tracker(servaddr);
	tracker.timer = Timer<double>{};
	bool lock = true;
	thread _thread = thread([&]() {
		cout << "created sending thread\n";

		while (lock) {
			Telemetry::Position position = drone.telemetry->position();

			send_buffer[0] = position.latitude_deg;
			send_buffer[1] = position.longitude_deg;
			send_buffer[2] = (double)position.absolute_altitude_m;

			if (1 < tracker.timer.toc().count()) {
				tracker.timer.tic();
				printf("send buffer\n");
				print_double_array(send_buffer, BUFFER_SIZE); // Drone's send gps coordinates to eachother.
			}
			// cout << "sending\n";
			// print_double_array(send_buffer, BUFFER_SIZE); // Drone's send gps coordinates to eachother.
			sendto(
				sockfd,
				(const char *)send_buffer,
				sizeof(send_buffer),
				MSG_CONFIRM,
				(const struct sockaddr *) &servaddr, 
				sizeof(servaddr)
			);
			sleep_for(milliseconds(100));
		}
	});
	tracker._thread = &_thread;
		

	Timer<double> main_timer{};
	fd_set readfds;
	int nbytes;
	int rv;
	struct timeval tv;
	tv.tv_sec = 10;
	tv.tv_usec = 0;
	// if (!fcntl(sockfd, F_SETFL, O_NONBLOCK)) return;
	fcntl(sockfd, F_SETFL, O_NONBLOCK);

	while (true) {
		// if ( 0 < thread_trackers.size() ) main_timer.tic(); // restarts timer
		// cout << "main_timer: " << main_timer.toc().count() << "\n";

		FD_ZERO(&readfds);
		FD_SET(sockfd, &readfds);

		rv = select(
			sockfd + 1,
			&readfds,
			NULL,
			NULL,
			&tv
		);

		if (rv == 1) {
			nbytes = recvfrom(
				sockfd,
				(char *)receive_buffer,
				sizeof(receive_buffer),
				MSG_WAITALL,
				( struct sockaddr *) &cliaddr,
				(&len)
			);
			cout << "receive buffer:\n";
			print_double_array(receive_buffer, BUFFER_SIZE);
		} else if (rv == 0) {
			cout << "haven't received packet in 10s\n";
			cout << "killing all threads\n";
			lock = false;
			break;
		}
		cout << "waiting for full message\n";
		n = recvfrom(
			sockfd,
			(char *)receive_buffer,
			sizeof(receive_buffer), 
			MSG_WAITALL,
			(struct sockaddr *) &servaddr,
			&len
		);

		cout << "receive_buffer:\n";
		print_double_array(receive_buffer, BUFFER_SIZE);
		// sleep_for(std::chrono::seconds(120));

		std::tuple<float, float, float> ned = drone.gpsa_to_ned(
			receive_buffer[0],
			receive_buffer[1],
			(float)receive_buffer[2]
		);

		std::get<0>(ned) = std::get<0>(ned) + stof(flag_args["--north"]);
		std::get<1>(ned) = std::get<1>(ned) + stof(flag_args["--east"]);
		std::get<2>(ned) = std::get<2>(ned) + stof(flag_args["--down"]);

		cout << "modified ned\n";
		cout << "north: " << std::get<0>(ned) << "\n";
		cout << "east: " << std::get<1>(ned) << "\n";
		cout << "down: " << std::get<2>(ned) << "\n";

		std::tuple<double, double, float> gpsa = drone.ned_to_gpsa(
			std::get<0>(ned),        // north
			std::get<1>(ned),        // east
			std::get<2>(ned)        // down
		);
		// // drone.unlock();
		// // std::get<2>(ned) = std::get<2>(ned) - 5.0f;
		cout << "latitude: " << std::get<0>(gpsa) << "\n";
		cout << "longitude: " << std::get<1>(gpsa) << "\n";
		cout << "altitude: " << std::get<2>(gpsa) << "\n";

		// drone.action_goto_gpsay_async(
		// 	std::get<0>(gpsa),
		// 	std::get<1>(gpsa),
		// 	std::get<2>(gpsa),
		// 	stof(flag_args["--yaw"]),
		// 	100.0f
		// );
		drone.offboard_goto_gpsay_async(
			std::get<0>(gpsa),
			std::get<1>(gpsa),
			std::get<2>(gpsa),
			stof(flag_args["--yaw"]),
			100.0f
		);

		// bool reached_destination = drone.action_goto_nedy(
		// 	std::get<0>(ned), // north
		// 	std::get<1>(ned),        // east
		// 	std::get<2>(ned),        // down
		// 	receive_buffer[3],       // yaw
		// 	100.0f                    // speed
		// );
		// sleep_for(milliseconds(100));
		// sleep_for(seconds(1));
	}

	tracker._thread->join();
	close(sockfd);

	return 0;
}
