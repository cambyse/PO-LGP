#ifndef _KINECT_H_
#define _KINECT_H_

#include <Core/module.h>
#include <functional>

#define Kinect_image_width 640
#define Kinect_image_height 480

struct KinectPoller : Module {
  struct sKinectInterface *s;

  ACCESS(byteA, kinect_rgb)
  ACCESS(MT::Array<uint16_t>, kinect_depth)

  KinectPoller();
  virtual ~KinectPoller();

  void open();
  void step();
  void close();
};

struct Kinect2PointCloud: Module {
  ACCESS(byteA, kinect_rgb)
  ACCESS(MT::Array<uint16_t>, kinect_depth)

  ACCESS(arr, kinect_points);
  ACCESS(arr, kinect_pointColors);

  arr pts,cols;
  floatA depth;
  byteA rgb; //helpers

  Kinect2PointCloud():Module("Kinect2PointCloud"){};
  virtual ~Kinect2PointCloud(){};

  void open(){};
  void step();
  void close(){};
};

namespace MLR {
    // convert raw image data into depth and color arrays like in a pointcloud
    void images2pointcloud(byteA& rgb, MT::Array<uint16_t>& depth, arr& pts, arr& cols);
	// pack 16bit depth image into 3 8-bit channels
	void pack_kindepth2rgb(const MT::Array<uint16_t>& depth, byteA& buffer);

	/// Typedef for depth image received event callbacks
	typedef std::function<void(const MT::Array<uint16_t>&, double)> kinect_depth_cb;
	/// Typedef for video image received event callbacks
	typedef std::function<void(const byteA&, double)> kinect_video_cb;

	class KinectCallbackReceiver {
	private:
		struct sKinectCallbackReceiver *s;
		int cameraNum;
	public:
		KinectCallbackReceiver(kinect_depth_cb depth_cb, kinect_video_cb video_cb, int cameraNum=0);
		virtual ~KinectCallbackReceiver();

		void startStreaming();
		void stopStreaming();
	};
}

BEGIN_MODULE(KinectDepthPacking)
    ACCESS(MT::Array<uint16_t>, kinect_depth);
    ACCESS(byteA, kinect_depthRgb);
END_MODULE()

#endif
