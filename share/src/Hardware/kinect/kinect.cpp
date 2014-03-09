#include "kinect.h"
#include <libfreenect.hpp>
#include <Core/util.h>

void lib_hardware_kinect(){ MT_MSG("loading"); }

REGISTER_MODULE(KinectPoller)
REGISTER_MODULE(Kinect2PointCloud)

const unsigned int image_width = 640; //kinect resolution
const unsigned int image_height = 480; //kinect resolution
const unsigned int depth_size = image_width*image_height;

//===========================================================================
//
// C++ interface to kinect: overloading callbacks that directly access the variables
//

struct sKinectInterface : Freenect::FreenectDevice {
  KinectPoller *module;

  sKinectInterface(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index), module(NULL) {
  }

  void DepthCallback(void *depth, uint32_t timestamp) {
    // use receive time, and subtract processing and communication delay of 120ms (experimentally determined)
    double tstamp = MT::clockTime() - .12;

    module->kinect_depth.writeAccess();
    memmove(module->kinect_depth().p, depth, 2*image_width*image_height);
    module->kinect_depth.tstamp() = tstamp;
    module->kinect_depth.deAccess();
  }

  void VideoCallback(void *rgb, uint32_t timestamp) {
    // see above
    double tstamp = MT::clockTime() - .12;

    module->kinect_rgb.writeAccess();
    memmove(module->kinect_rgb().p, rgb, 3*image_width*image_height);
    module->kinect_rgb.tstamp() = tstamp;
    module->kinect_rgb.deAccess();
  }
};


//===========================================================================
//
// Poller
//

Freenect::Freenect *freenect = NULL;

KinectPoller::KinectPoller() : Module("KinectInterface"), s(NULL){
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
  s = &(freenect->createDevice<sKinectInterface>(0));
  s->module = this;

  // The following is only available for newer versions of libfreenect
  // (newer versions actually also have a much improved interface)
  //int ret;
  //ret = freenect_set_flag(s->getDevice(), FREENECT_AUTO_EXPOSURE, FREENECT_OFF);
  //if(ret != 0)
    //cout << "freenect_set_flag failed!" << endl;
  //else
    //cout << "freenect_set_flag worked!" << endl;
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

//===========================================================================
//
// converter to point cloud
//

void Kinect2PointCloud::step(){
  copy(depth, kinect_depth.get()());
  rgb = kinect_rgb.get();

  if(depth.N!=image_width*image_height || rgb.N!=3*image_width*image_height){
    MT_MSG("here" <<depth.getDim() <<' ' <<kinect_depth.get()->getDim());
    return;
  }

  rgb.reshape(image_width*image_height, 3);
  pts.resize(image_width*image_height, 3);
  cols.resize(image_width*image_height, 3);

  float constant = 1.0f / 580; //focal length of kinect in pixels
  int centerX = (image_width >> 1);
  int centerY = (image_height >> 1);

  int value_idx = 0;
  int point_idx = 0;
  for (int v = -centerY; v < centerY; ++v) {
    for (int u = -centerX; u < centerX; ++u, ++value_idx, ++point_idx) {
      double d=depth.elem(value_idx);
      if (d!= 0 && d!=2047) {
        double z=(double) d * 0.001;
        pts(point_idx, 0) = z*constant*u;
        pts(point_idx, 1) = z*constant*v;
        pts(point_idx, 2) = z;

        cols(point_idx, 0) = (double)rgb(point_idx, 0)/255.;
        cols(point_idx, 1) = (double)rgb(point_idx, 1)/255.;
        cols(point_idx, 2) = (double)rgb(point_idx, 2)/255.;
      }
    }
  }

  kinect_points.set() = pts;
  kinect_pointColors.set() = cols;
}

