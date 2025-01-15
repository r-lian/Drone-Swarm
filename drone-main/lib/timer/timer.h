#ifndef __MY_TIMER_H__

#include <chrono>

using namespace std::chrono;

#define __MY_TIMER_H__

template <class Rep>
class Timer {
private:
	time_point<steady_clock> start;
	time_point<steady_clock> end;
	duration<Rep> dur;
public:
	Timer();
	void tic();
	duration<Rep> toc();
	duration<Rep> get_duration();
};


// Function templates cannot be compiled.
// Specific types for functions are found in used source.
template<class Rep>
Timer<Rep>::Timer(){
	start = steady_clock::now();
	end = steady_clock::now();
}

template<class Rep>
void Timer<Rep>::tic() {
	start = steady_clock::now();
}

template<class Rep>
duration<Rep> Timer<Rep>::toc() {
	end = steady_clock::now();
	dur = end - start;
	return dur;
}

template<class Rep>
duration<Rep> Timer<Rep>::get_duration() {
	return dur;
}

#endif
