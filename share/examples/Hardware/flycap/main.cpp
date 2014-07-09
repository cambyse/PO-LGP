#include <Hardware/flycap/flycap.h>
#include <Core/array.h>
#include <Gui/opengl.h>
#include <iostream>
#include <functional>

using namespace std;
using namespace std::placeholders;
using namespace MLR;

int main(int argc, char* argv[])
{
	int serial = 0;
	if(argc > 1) {
		serial = strtol(argv[1], NULL, 10);
	}

	cout << "Opening camera with serial number " << serial << endl;

	try {
		OpenGL gl;
		FlycapInterface flycap(serial);
		flycap.startStreaming();
	
		double start;
		start = MT::clockTime();
		unsigned int count = 0;
		while(true) {
			double timestamp;
			if(flycap.grab(gl.background, timestamp)) {
				gl.update();
				++count;
			}
			if((count % 100) == 0) {
				double now = MT::clockTime();
				double per_frame = ((now - start) / 100.0);
				clog << "Frame time: " << per_frame << " -- fps: " << (1.0/per_frame) << endl;
				start = now;
			}
		}
	} catch(const std::exception& ex) {
		cerr << ex.what() << endl;
	}

	return 0;
}

