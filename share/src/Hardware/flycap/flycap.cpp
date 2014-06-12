#include <Core/thread.h>
#include <Core/util.h>
#include <flycapture/FlyCapture2.h>
#include "flycap.h"

REGISTER_MODULE(FlycapPoller)

void lib_hardware_flycapcamera() { cout << "loading flycapcamera" << endl; }

//const unsigned int c_flycap_width = 1280;
//const unsigned int c_flycap_height = 1024;
const unsigned int c_flycap_width = 1280;
const unsigned int c_flycap_height = 1024;
const unsigned int c_flycap_fps = 60;
const unsigned int c_flycap_bpp = 24;
const unsigned int c_flycap_bypp = c_flycap_bpp / 8;
const unsigned int c_flycap_size = c_flycap_width * c_flycap_height * c_flycap_bypp;

using namespace FlyCapture2;
using namespace MLR;

//===========================================================================
//
// C++ interface to flycap
//

TStream tout(cout);

namespace {
	void CHECK_ERROR(const Error e) {
		if(e != PGRERROR_OK)
			throw FlycapException(e.GetDescription());
	}
}

struct sFlycapInterface {
	Camera cam;

	sFlycapInterface(int cameraID) {
		BusManager bm;
		PGRGuid *id;
		CHECK_ERROR(bm.GetCameraFromIndex(cameraID, id));
		CHECK_ERROR(cam.Connect(id));
	}
	~sFlycapInterface() {
		cam.Disconnect();
	}

	void start() {
		cam.StartCapture();
	}
	void stop() {
		cam.StopCapture();
	}

	bool grab(byteA& image, double& timestamp, unsigned int timeout=1<<31) {
		Image img(image.p, c_flycap_size);
		Error e = cam.RetrieveBuffer(&img);
		if(e == PGRERROR_OK) {
			// TODO: use more accurate embedded timestamp
			TimeStamp ts(img.GetTimeStamp());
			timestamp = (double)ts.cycleSeconds + (((double)ts.microSeconds) / 1e6);
			return true;
		} else {
			tout << "Could not grab image: " << e;
			return false;
		}
	}

};



//===========================================================================
//
// Poller
//

FlycapPoller::FlycapPoller() : Module("FlycapInterface"), s(NULL) {
}

FlycapPoller::~FlycapPoller() {
}

void FlycapPoller::open() {
}

void FlycapPoller::step() {
	Access_typed<byteA>::WriteToken token(&rgb);
	s->grab(rgb(), rgb.tstamp());
}

void FlycapPoller::close() {
  delete s;
  tout(this) << "closed successfully" << endl;
}

namespace MLR {
	Mutex start_lock;

	FlycapInterface::FlycapInterface(int cameraID) : s(new ::sFlycapInterface(cameraID)), streaming(false) {

	}
	FlycapInterface::~FlycapInterface() {

		delete s;
	}
	void FlycapInterface::startStreaming() {

	}
	bool FlycapInterface::grab(byteA& image, double& timestamp, unsigned int timeout) {
		return s->grab(image, timestamp, timeout);
	}
}
