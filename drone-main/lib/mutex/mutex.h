#ifndef MY_MUTEX_H

#define MY_MUTEX_H

#include <mutex>

class Mutex {
protected:
	std::mutex mut;
	bool _is_locked = false;
public:
	void lock();
	void unlock();
	bool is_locked();
};
#endif
