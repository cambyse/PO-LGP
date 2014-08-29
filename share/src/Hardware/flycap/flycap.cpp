#include <Core/thread.h>
#include <Core/util.h>
#include <flycapture/FlyCapture2.h>
#include "flycap.h"
#include <devTools/logging.h>
#include <iomanip>
#include <deque>
#include <condition_variable>
#include <array>

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

	FlyCapture2::PixelFormat mlr2fc(MLR::PixelFormat format) {
		switch(format) {
		case MLR::PIXEL_FORMAT_RAW8:
			return FlyCapture2::PIXEL_FORMAT_RAW8;
		case MLR::PIXEL_FORMAT_BGR8:
			return FlyCapture2::PIXEL_FORMAT_BGR;
		case MLR::PIXEL_FORMAT_RGB8:
			return FlyCapture2::PIXEL_FORMAT_RGB8;
		case MLR::PIXEL_FORMAT_YUV444_8:
			throw FlycapException("Pixel format yuv444_8 not natively supported by Flycap 2");
		case MLR::PIXEL_FORMAT_UYV444:
			return FlyCapture2::PIXEL_FORMAT_444YUV8;
		case MLR::PIXEL_FORMAT_UYV422:
			return FlyCapture2::PIXEL_FORMAT_422YUV8;
		default:
			throw FlycapException("Specified pixel format not supported (no equivalent in flycap?)");
		}
	}

	struct ImageCapture {
		Image image;
		double timestamp;
		ImageCapture(const Image& im, double timestamp) : image(im), timestamp(timestamp) {}
		ImageCapture() : timestamp(0) {};
	};
}

namespace MLR {

void image_callback(Image *pImage, const void* callbackData);

struct sFlycapInterface {
	int id;
	GigECamera cam;
	Image targetImage;
	FlyCapture2::PixelFormat output_format;
	deque<ImageCapture> captured_images;
	mutex list_access;
	condition_variable cv;
	bool started;
	double first_capture_time, last_capture_time, capture_diffs;
	int count;

	sFlycapInterface(int cameraID, MLR::PixelFormat capture_fmt, MLR::PixelFormat output_fmt) : id(cameraID),
			started(false), first_capture_time(-1), last_capture_time(0), capture_diffs(0), count(0) {
		BusManager bm;
		PGRGuid id;
		CHECK_ERROR(bm.GetCameraFromSerialNumber(cameraID, &id));
		CHECK_ERROR(cam.Connect(&id));
		// configure camera
		FC2Config conf;
		conf.grabMode = BUFFER_FRAMES;
		conf.highPerformanceRetrieveBuffer = true;
		conf.numBuffers = 10;
		conf.isochBusSpeed = BUSSPEED_S_FASTEST;
		CHECK_ERROR(cam.SetConfiguration(&conf));

		cam.SetGigEImagingMode(MODE_0);

		GigEImageSettings settings;

		settings.width = 1280;
		settings.height = 1024;
		settings.pixelFormat = mlr2fc(capture_fmt);

		output_format = mlr2fc(output_fmt);
		targetImage.SetDimensions(1024, 1280, 0, output_format, NONE);
		CHECK_ERROR(cam.SetGigEImageSettings(&settings));

		GigEStreamChannel cinfo;
		cam.GetGigEStreamChannelInfo(0, &cinfo);
		cinfo.packetSize = 9000;
		cinfo.interPacketDelay = 0;
		cam.SetGigEStreamChannelInfo(0, &cinfo);

		EmbeddedImageInfo info;
		info.timestamp.onOff = true;
		CHECK_ERROR(cam.SetEmbeddedImageInfo(&info));

		Image::SetDefaultColorProcessing(IPP);
	}
	~sFlycapInterface() {
		cam.Disconnect();
	}

protected:
	void update_stats(double timestamp) {
		if(!started) {
			first_capture_time = timestamp;
			started = true;
		}
		++count;
		capture_diffs+=timestamp-last_capture_time;
		last_capture_time = timestamp;
	}
	void reset_stats(double timestamp) {
		count = 0;
		capture_diffs = 0;
		last_capture_time = timestamp;
		first_capture_time = timestamp;
	}


public:
	void add_image(Image* pImage) {
		double timestamp = clockTime();
		{
			unique_lock<mutex> lck(list_access);
			update_stats(timestamp);

			captured_images.push_back(ImageCapture(*pImage, timestamp));
			if(captured_images.size() > 50) {
				WARN(flycap, STRING("Capture queue growing, currently size " << captured_images.size()));
			}

			cv.notify_one();
		}

		if((count % 100) == 0) {
			print_stats();
			reset_stats(timestamp);
		}

	}

	void print_stats() {
		CameraStats stats;
		cam.GetStats(&stats);
		double elapsed = last_capture_time - first_capture_time;
		double fps = count / elapsed;
		cout << id << " running at " << fps << "fps, "
				<< " dropped " << stats.imageDriverDropped << "+" << stats.imageDropped
				<< ", xmit failed" << stats.imageXmitFailed << ", resend "
				<< stats.numResendPacketsReceived << "/" << stats.numResendPacketsRequested << endl;
	}

	void start() {
		cam.StartCapture(&image_callback, this);
		//cam.StartCapture();
	}
	void stop() {
		cam.StopCapture();
	}

	bool grab(byteA& image, double& timestamp, unsigned int timeout=1<<31) {
//		Image img;
//		cam.RetrieveBuffer(&img);
//		update_stats(clockTime());
//
//		if((count % 100) == 0) {
//			print_stats();
//		}
//		return true;

		ImageCapture ic;
		{
			unique_lock<mutex> lck(list_access);
			if(captured_images.size() < 1) {
				cv.wait(lck);
			}

			ic = captured_images.front();
			captured_images.pop_front();
		}
		timestamp = ic.timestamp;
//		return true;

		image.resize(c_flycap_height, c_flycap_width, 3);
		targetImage.SetData(image.p, c_flycap_size);
		ic.image.Convert(output_format, &targetImage);

		return true;
	}
};

void image_callback(Image *pImage, const void* callbackData) {
	sFlycapInterface* cap = const_cast<sFlycapInterface*>((sFlycapInterface*)callbackData);
	cap->add_image(pImage);
}

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

FlycapInterface::FlycapInterface(int cameraID, MLR::PixelFormat capture_fmt, MLR::PixelFormat output_fmt) :
		s(new sFlycapInterface(cameraID, capture_fmt, output_fmt)), streaming(false) {

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
	Access_typed<byteA>::WriteToken token(&image);
	s->grab(image(), image.tstamp());
}

void FlycapPoller::close() {
  delete s;
  tout(this) << "closed successfully" << endl;
}

