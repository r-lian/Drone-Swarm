#include "thread_tracker.h"

thread_tracker::thread_tracker(struct sockaddr_in addr) {
	lock = true;
	ip_address = addr.sin_addr.s_addr;
	port = addr.sin_port;
}

bool operator<(const struct thread_tracker &lh, const struct thread_tracker &rh) {
	if ( lh.ip_address < rh.ip_address ) return true;
	if ( lh.ip_address > rh.ip_address ) return false;
	return lh.port < rh.port;
}

bool operator<(struct thread_tracker &lh, struct thread_tracker &rh) {
	if ( lh.ip_address < rh.ip_address ) return true;
	if ( lh.ip_address > rh.ip_address ) return false;
	return lh.port < rh.port;
}

bool operator==(const struct thread_tracker &lh, const struct thread_tracker &rh) {
	return (
		(lh.ip_address == rh.ip_address) &&
		(lh.port == rh.port)
	);
}

bool operator==(struct thread_tracker &lh, struct thread_tracker &rh) {
	return (
		(lh.ip_address == rh.ip_address) &&
		(lh.port == rh.port)
	);
}

bool operator==(struct thread_tracker &lh, struct sockaddr_in &rh) {
	return (
		(lh.ip_address == rh.sin_addr.s_addr) &&
		(lh.port == rh.sin_port)
	);
}
