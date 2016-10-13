#include <unistd.h>
#include <time.h>
#include <iostream>
#include <Core/thread.h>

using namespace std;

#define COUNT 1000

double start = 0;

namespace {
	double as_sec(const timespec &ts) {
		return ts.tv_sec + (ts.tv_nsec / 1e9) - start;
	}
}

int main(int argc, char* argv[]) {
	unsigned int sleep_usec = 1000; // 1ms
	double sleep_sec = sleep_usec / 1e6;

	double diffs_to_target = 0, slept = 0;
	timespec before, after;

	clock_gettime(CLOCK_REALTIME, &before);
	start = as_sec(before);

	Metronome m("t", sleep_sec);

	for(unsigned int i = 0; i < COUNT; ++i) {
		clock_gettime(CLOCK_REALTIME, &before);
		m.waitForTic();
		clock_gettime(CLOCK_REALTIME, &after);

		//cout << "before " << as_sec(before) << ", after " << as_sec(after) << endl;
		double diff = as_sec(after) - as_sec(before);
		//cout << "diff " << diff << endl;
		slept+=diff;
		diff-=sleep_sec;
		diffs_to_target+=diff;
	}

	cout << "Target sleep time " << (COUNT*sleep_sec) << "s" << endl
			<< "Actual time " << slept << "s" << endl
			<< "Jitter " << ((diffs_to_target/(double)COUNT)*1e3) << "ms" << endl;

}
