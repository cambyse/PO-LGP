#include "kinect.h"
#include <libfreenect.hpp>
#include <Core/util.h>

void lib_hardware_kinect(){ MT_MSG("loading"); }

REGISTER_MODULE(KinectPoller)

const unsigned int image_width = 640; //kinect resolution
const unsigned int image_height = 480; //kinect resolution
const unsigned int depth_size = image_width*image_height;

namespace MLR {
	Freenect::Freenect receiver_freenect;

	class sKinectCallbackReceiver : public Freenect::FreenectDevice {
	private:
		kinect_depth_cb depth_cb;
		kinect_video_cb video_cb;
		bool streaming;
	public:
		sKinectCallbackReceiver(freenect_context *ctx, int index) :
			Freenect::FreenectDevice(ctx, index), depth_cb(nullptr), video_cb(nullptr), streaming(false) {
			freenect_device_attributes *attrs = NULL;
			int num_attrs = freenect_list_device_attributes(ctx, &attrs);
			for(int i = 0; i < num_attrs; i++, attrs = attrs->next) {
				std::cout << "Serial " << i << "=" << attrs->camera_serial << std::endl;
			}
		}
		~sKinectCallbackReceiver() {
			stopStreaming();
		}
		void add_callbacks(kinect_depth_cb depth_cb, kinect_video_cb video_cb) {
			this->depth_cb = depth_cb;
			this->video_cb = video_cb;
		}
		void startStreaming() {
			if(!streaming) {
				startVideo();
				startDepth();
				setDepthFormat(FREENECT_DEPTH_REGISTERED);
				streaming = true;
			}
		}
		void stopStreaming() {
			if(streaming) {
				stopVideo();
				stopDepth();
				streaming = false;
			}
		}

		void DepthCallback(void *depth, uint32_t ) {
			uint16A depth_buf((uint16_t*)depth, depth_size);
			depth_buf.reshape(image_height, image_width);
			double timestamp = MT::clockTime();// - .12;
			if(depth_cb != nullptr)
				depth_cb(depth_buf, timestamp);
		}
		void VideoCallback(void *rgb, uint32_t ) {
			byteA video_buf((byte*)rgb, depth_size*3);
			video_buf.reshape(image_height, image_width, 3);
			double timestamp = MT::clockTime(); // - .12;
			if(video_cb != nullptr)
				video_cb(video_buf, timestamp);
		}
	};


	KinectCallbackReceiver::KinectCallbackReceiver(kinect_depth_cb depth_cb, kinect_video_cb video_cb,
			int cameraNum) : s(&(receiver_freenect.createDevice<sKinectCallbackReceiver>(cameraNum))),
					cameraNum(cameraNum) {
		s->add_callbacks(depth_cb, video_cb);
	}
	KinectCallbackReceiver::~KinectCallbackReceiver() {
		receiver_freenect.deleteDevice(cameraNum);
		s = NULL;
	};
	void KinectCallbackReceiver::startStreaming() {
		s->startStreaming();
	}
	void KinectCallbackReceiver::stopStreaming() {
		s->stopStreaming();
	}
}

//===========================================================================
//
// C++ interface to kinect: overloading callbacks that directly access the variables
//

struct sKinectPoller : Freenect::FreenectDevice {
  KinectPoller *module;

  sKinectPoller(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index), module(NULL) {
  }

  void DepthCallback(void *depth, uint32_t timestamp) {
    memmove(module->kinect_depth.set()->p, depth, 2*image_width*image_height);
    // use receive time, and subtract processing and communication delay of 120ms (experimentally determined)
    module->kinect_depth.tstamp() = MT::clockTime() - .12;
  }

  void VideoCallback(void *rgb, uint32_t timestamp) {
    memmove(module->kinect_rgb.set()->p, rgb, 3*image_width*image_height);
    // see above
    module->kinect_rgb.tstamp() = MT::clockTime() - .12;
  }
};


//===========================================================================
//
// Poller
//

Freenect::Freenect *freenect = NULL;

KinectPoller::KinectPoller() : Module("KinectPoller"), s(NULL){
}

KinectPoller::~KinectPoller() {
  if(freenect) delete freenect;
  freenect=NULL;
}

void KinectPoller::open() {
  cout <<"KinectPoller opening..." <<endl;
  kinect_rgb.set()->resize(image_height, image_width, 3);
  kinect_depth.set()->resize(image_height, image_width);

  if(!freenect) freenect = new Freenect::Freenect;
  s = &(freenect->createDevice<sKinectPoller>(0));
  s->module = this;

  s->startVideo();
  s->startDepth();
  s->setDepthFormat(FREENECT_DEPTH_REGISTERED);  // use hardware registration
}

void KinectPoller::step() {
  //s->updateState(); //actually I think this step routine is redundant because the callback access the variable and fire its revision
}

void KinectPoller::close() {
  s->stopVideo();
  s->stopDepth();
  freenect->deleteDevice(0);
  s = NULL;
}
