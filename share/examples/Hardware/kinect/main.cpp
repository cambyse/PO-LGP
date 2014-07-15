#include <Hardware/kinect/kinect.h>
#include <Core/array.h>
#include <Gui/opengl.h>
#include <iostream>
#include <functional>

using namespace std;
using namespace std::placeholders;

namespace {
	void video_cb(OpenGL& gl, const byteA& rgb, double timestamp) {
		gl.watchImage(rgb, true, 1.0);
	}

}

int main(int argc, char* argv[])
{
	OpenGL gl;
	MLR::kinect_video_cb cb = std::bind(&video_cb, std::ref(gl), _1, _2);

	MLR::KinectCallbackReceiver receiver(nullptr,
			[&gl](const byteA& video, double){ gl.watchImage(video, false, 1.0); }, 0);

	receiver.startStreaming();
	MT::wait(5.);
	receiver.stopStreaming();

	return 0;
}

