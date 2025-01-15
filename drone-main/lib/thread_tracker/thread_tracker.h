#ifndef __THREAD_TRACKER_H__
#define __THREAD_TRACKER_H__

#include <thread>
#include <timer/timer.h>
#include <arpa/inet.h>

using namespace std;

typedef struct thread_tracker {
	bool lock = true;
	thread *_thread;
	Timer<double> timer;
	unsigned long ip_address;
	unsigned short port;

	thread_tracker(struct sockaddr_in addr);
} thread_tracker;

bool operator<(const struct thread_tracker &lh, const struct thread_tracker &rh);
bool operator<(struct thread_tracker &lh, struct thread_tracker &rh);
bool operator==(const struct thread_tracker &lh, const struct thread_tracker &rh);
bool operator==(struct thread_tracker &lh, struct thread_tracker &rh);
bool operator==(struct thread_tracker &lh, struct sockaddr_in &rh);

#endif
