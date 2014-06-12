#include <Core/thread.h>
#include <Core/util.h>
#include <flycapture/FlyCapture2.h>
#include "flycap.h"
#include <devTools/logging.h>

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
using namespace MT;
using namespace std;

SET_LOG(flycap, LogLevel::INFO)

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

namespace MLR {

struct sFlycapInterface {
	Camera cam;

	sFlycapInterface(int cameraID) {
		BusManager bm;
		PGRGuid id;
		CHECK_ERROR(bm.GetCameraFromSerialNumber(cameraID, &id));
		CHECK_ERROR(cam.Connect(&id));
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
		Image img;
		Error e = cam.RetrieveBuffer(&img);
		if(e == PGRERROR_OK) {
			image.resize(img.GetRows(), img.GetCols(), img.GetBitsPerPixel() / 8);
			memcpy(image.p, img.GetData(), img.GetRows() * img.GetCols() * img.GetBitsPerPixel() / 8);
			// TODO: use more accurate embedded timestamp
			TimeStamp ts(img.GetTimeStamp());
			timestamp = (double)ts.cycleSeconds + (((double)ts.microSeconds) / 1e6);
			return true;
		} else {
			ERROR(flycap, STRING("Could not grab image: " << e.GetDescription()));
			return false;
		}
	}
};

vector<uint32_t> get_flycap_ids() {
	BusManager bus;
	unsigned int num_cams;
	CHECK_ERROR(bus.GetNumOfCameras(&num_cams));
	
	vector<uint32_t> result;
	for(unsigned int i = 0; i < num_cams; ++i) {
		unsigned int serial;
		CHECK_ERROR(bus.GetCameraSerialNumberFromIndex(i, &serial));
		result.push_back(serial); 
	}
	return result;
}



Mutex start_lock;

FlycapInterface::FlycapInterface(int cameraID) : s(new sFlycapInterface(cameraID)), streaming(false) {

}
FlycapInterface::~FlycapInterface() {
	s->stop();
	delete s;
}
void FlycapInterface::startStreaming() {
	if(!streaming) {
		s->start();
		streaming = true;
	}
}
bool FlycapInterface::grab(byteA& image, double& timestamp, unsigned int timeout) {
	startStreaming();	
	return s->grab(image, timestamp, timeout);
}
}

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

