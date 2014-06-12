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
	
		while(true) {
			double timestamp;
			if(flycap.grab(gl.background, timestamp)) {
				gl.update();
			}
		}
	} catch(const std::exception& ex) {
		cerr << ex.what() << endl;
	}

	return 0;
}

