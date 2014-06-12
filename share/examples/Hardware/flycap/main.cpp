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
	OpenGL gl;
	FlycapInterface flycap(0);
	flycap.startStreaming();
	
	byteA rgb;
	while(true) {
		double timestamp;
		flycap.grab(rgb, timestamp);
		gl.update();
	}


	return 0;
}

