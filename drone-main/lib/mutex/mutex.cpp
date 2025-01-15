#include "mutex.h"

void Mutex::lock() {
	_is_locked = true;
	mut.lock();
}

void Mutex::unlock() {
	_is_locked = false;
	mut.unlock();
}

bool Mutex::is_locked() {
	return _is_locked;
}
