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
		start = mlr::clockTime();
		unsigned int count = 0;
		while(true) {
			double timestamp;
			if(flycap.grab(gl.background, timestamp)) {
				double w_ratio = (double)gl.width / (double)gl.background.d0;
				double h_ratio = (double)gl.height / (double)gl.background.d1;
				gl.backgroundZoom = min(w_ratio, h_ratio);
				gl.update(NULL, false, false, false);
				++count;
			}
			if((count % 100) == 0) {
				double now = mlr::clockTime();
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

