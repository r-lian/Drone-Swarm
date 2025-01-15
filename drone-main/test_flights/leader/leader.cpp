// Server side implementation of UDP client-server model

#include "helpers.h"

#define PORT        8080
#define MAXLINE     1024
#define BUFFER_SIZE    4

using namespace std;
using std::chrono::seconds;
using std::this_thread::sleep_for;


void fill_server_address_struct(struct sockaddr_in &servaddr) {
	// Filling server information
	servaddr.sin_family    = AF_INET; // IPv4
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(PORT);
}

void create_socket(int &sockfd, struct sockaddr_in &servaddr) {
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
}

bool wait_for_packet_with_timeout(time_t timeout_s, int sockfd) {
	fd_set readfds;
	int rv;
	struct timeval tv;
	tv.tv_sec = timeout_s;
	tv.tv_usec = 0;

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
		return true;
	} else if (rv == 0) {
		cout << "waiting for packet has timed out\n";
	} else {
		cout << "error while waiting for packet\n";
	}
	cout << "packet was not received\n";
	return false;
}

void client_thread(
	int sockfd,
	Drone *drone,
	thread_tracker *_client_thread_tracker,
	set<thread_tracker>::iterator _client_thread_tracker_iterator,
	set<thread_tracker> *_thread_trackers,
	struct sockaddr_in _cliaddr
) {
	 cout << "created thread\n";

	 double send_buffer[BUFFER_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f };
	 double send_timeout = 1.0;

	 while (_client_thread_tracker->lock) {
	 	if ( send_timeout < _client_thread_tracker->timer.toc().count() ) {
	 		cout << "thread timed out\n";
	 		break;
	 	}

	 	Telemetry::Position position = drone->telemetry->position();

	 	send_buffer[0] = position.latitude_deg;
	 	send_buffer[1] = position.longitude_deg;
	 	send_buffer[2] = (double)position.absolute_altitude_m;

	 	// cout << "send buffer:\n";
	 	// print_double_array(send_buffer, BUFFER_SIZE);
	 	sendto(
	 		sockfd,
	 		(const char *)send_buffer,
	 		sizeof(send_buffer), 
	 		MSG_CONFIRM,
	 		(const struct sockaddr *) &_cliaddr,
	 		sizeof(_cliaddr)
	 	);
	 	// cout << "thread going to sleep\n";
	 	// sleep_for(chrono::milliseconds(10));
	 	// sleep_for(chrono::seconds(1));
	 }
	 // delete _client_thread_tracker->_thread;
	 _thread_trackers->erase(_client_thread_tracker_iterator);
	 cout << "exiting thread\n";
}


int main(int argc, char *argv[]) {
	cout.precision(17);
	// Handles Command Line Arguments
	Command_Line_Argument_Parser parser(argc, argv);
	map<string, string> flag_args = parser.get_flag_arguments();

	// If no connection url tries to connect with empty string.
	Drone drone(flag_args["--connection_url"]);
	cout << "drone constructed.\n";

	if ( !drone.is_connected_to_flight_controller() ) {
		cout << parser.get_command() << " "
			<< connection_url_flag << "\n"
			<< connection_url_format;

		return -1;
	}

	double receive_buffer[BUFFER_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f };
	int sockfd;
	struct sockaddr_in servaddr, cliaddr;

	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));

	fill_server_address_struct(servaddr);
	create_socket(sockfd, servaddr);

	int n;
	socklen_t len = sizeof(cliaddr);

	set<thread_tracker> thread_trackers;
	set<thread *> client_threads;

	Timer<double> main_timer{};
	while (main_timer.toc().count() < 30.0) {
		if ( 0 < thread_trackers.size() ) {
			cout << "thread_trackers.size(): " << thread_trackers.size() << "\n";
			main_timer.tic();
		}

		// int counter = 0;
		// for (
		// 	auto iter = thread_trackers.begin();
		// 	iter != thread_trackers.end();
		// ) {
		// 	thread_tracker &t = const_cast<thread_tracker &>(*iter);
		// 	cout << "Checking thread timer: " << ++counter << "\n";
		// 	if ( 1.0 < t.timer.toc().count() ) {
		// 		cout << "thread timer " << counter << " timed out\n";
		// 		t.lock = false;
		// 		t._thread->join();
		// 		delete t._thread;
		// 		iter = thread_trackers.erase(iter);
		// 		cout << "thread_trackers.size(): " << thread_trackers.size() << "\n";
		// 		cout << "deleted thread memory\n";
		// 		continue;
		// 	}
		// 	iter++;
		// }

		if (wait_for_packet_with_timeout(1, sockfd)) {
			int nbytes = recvfrom(
				sockfd,
				(char *)receive_buffer,
				sizeof(receive_buffer),
				MSG_WAITALL,
				( struct sockaddr *) &cliaddr,
				(&len)
			);
			cout << "main_timer timeout: " << main_timer.toc().count() << "\n";

			if (nbytes == -1) {
				cout << "error while receiving packet\n";
				continue;
			}
			// cout << "receive buffer:\n";
			// print_double_array(receive_buffer, BUFFER_SIZE);
		} else {
			cout << "main_timer timeout: " << main_timer.toc().count() << "\n";
			cout << "wait for packet has timed out\n";
			continue;
		}

		pair<set<thread_tracker>::iterator, bool> ret = thread_trackers.insert(thread_tracker(cliaddr));

		bool tracker_exists = ret.second;
		set<thread_tracker>::iterator client_thread_tracker_iterator = ret.first;
		thread_tracker &client_thread_tracker = const_cast<thread_tracker &>(*(ret.first));

		if ( !tracker_exists ) {
			cout << "updating time for thread tracker: " << client_thread_tracker.ip_address << ":" << client_thread_tracker.port << "\n";
			 // Done from the set because set.insert stores a copy.
			client_thread_tracker.timer.tic();
			continue;
		}
		cout << "inserted new thread tracker: " << client_thread_tracker.ip_address << ":" << client_thread_tracker.port << "\n";

		cout << "creating thread\n";
		// Needs to delete the memory when done. But how?
		client_thread_tracker._thread = new thread(
			client_thread,
			sockfd,
			&drone,
			&client_thread_tracker ,
			client_thread_tracker_iterator,
			&thread_trackers,
			cliaddr
		);
		client_thread_tracker._thread->detach();
		client_threads.insert(client_thread_tracker._thread);

		// sleep_for(seconds(1));
		// cout << "end of while loop\n";
	}
	cout << "outside return\n";
	int counter = 1;
	while ( !client_threads.empty() ) {
		cout << "deleting client thread " << counter++ << "\n";
		set<thread *>::iterator iter = client_threads.begin();
		thread *t = *iter;
		delete t;
		iter = client_threads.erase(iter);
	}

	return 0;
}
