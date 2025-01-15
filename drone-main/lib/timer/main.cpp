#include <iostream>
#include "timer.h"

using namespace std;
using namespace std::chrono;

int main() {
	Timer<float> timer{};
	timer.tic();
	for (int i = 0; i < 1000000 ; i++);
	// duration<long int> dur = duration_cast<nanoseconds>(timer.toc());
	// duration<float> dur = duration_cast<milliseconds>(timer.toc());
	duration<float, nano> dur = timer.toc();


	// auto dur = duration_cast<milliseconds>(timer.toc());
	cout << dur.count() << "\n";
}
