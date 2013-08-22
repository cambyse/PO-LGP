#ifndef _KINECT_H_
#define _KINECT_H_

#include <System/biros.h>

struct PointCloudVar : Variable {
  PointCloudVar(const char* name):Variable(name){}
  MT::Array<uint16_t> depth;
  MT::Array<uint8_t> rgb;
};

struct KinectInterface : Process {
  struct sKinectInterface *s;
  PointCloudVar *pointCloud;
    
  KinectInterface(const char *name);
  virtual ~KinectInterface();

  void open();
  void step();
  void close();
};

struct FreenectData : public Variable {
  FreenectData() : Variable("Freenect Data"),
                   buffer_depth(480, 640),
                   buffer_video(480, 640, 3),
                   new_rgb_frame(false),
                   new_depth_frame(false),
                   frame_id(0) {
  };

  MT::Array<uint16_t> buffer_depth;
  MT::Array<uint8_t> buffer_video;
  MT::Array<uint16_t> gamma;

  bool new_rgb_frame;
  bool new_depth_frame;

  uint frame_id;
};


#include <libfreenect.hpp>

struct sKinectInterface : Freenect::FreenectDevice {
  sKinectInterface(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index) {
    rgb_buffer.resize(image_height, image_width, 3);
  };

  static const unsigned int image_width;
  static const unsigned int image_height;
  byteA rgb_buffer;

  KinectInterface *p;
  FreenectData* data;

  void DepthCallback(void* _depth, uint32_t timestamp) {
    data->writeAccess(p->module);
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    data->buffer_depth.setCarray(depth, data->buffer_depth.N);
    data->new_depth_frame = true;
    data->deAccess(p->module);
  }

  void VideoCallback(void* _rgb, uint32_t timestamp) {
    data->writeAccess(p->module);
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    data->buffer_video.setCarray(rgb, data->buffer_video.N);
    data->new_rgb_frame = true;
    data->deAccess(p->module);
  }

  bool getRGB(MT::Array<uint8_t> &buffer) {
    data->writeAccess(p->module);
    if (!data->new_rgb_frame) {
      data->deAccess(p->module);
      return false;
    }
    buffer.swap(data->buffer_video);
    data->new_rgb_frame = false;
    data->deAccess(p->module);
    return true;
  }

  bool getDepth(MT::Array<uint16_t> &buffer) {
    data->writeAccess(p->module);
    if (!data->new_depth_frame) {
      data->deAccess(p->module);
      return false;
    }
    buffer.swap(data->buffer_depth);
    data->new_depth_frame = false;
    data->deAccess(p->module);
    return true;
  }
};

#endif
