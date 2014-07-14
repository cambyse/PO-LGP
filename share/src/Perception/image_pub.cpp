/*
 * image_pub.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: ingo
 */

#include "image_pub.h"

#ifdef HAVE_ROS_IMAGE_TRANSPORT
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
using namespace image_transport;
using namespace camera_info_manager;
#endif

#include <Core/array.h>
#include <sstream>

namespace MLR {

struct sImagePublisher {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
	ros::NodeHandle n;
	ImageTransport t;
	Publisher p;
	CameraInfoManager cim;
	sensor_msgs::Image msg;
#endif
	uint32_t seq, bypp;
	std::string camera_name, link_name, encoding;
	double epoch_offset;
	PixelFormat pix_fmt;

	sImagePublisher(const std::string& base_topic, const std::string& camera_name, PixelFormat pix_fmt) :
#ifdef HAVE_ROS_IMAGE_TRANSPORT
		n(base_topic), t(n), p(t.advertise(camera_name, 1)), cim(n, camera_name),
#endif
		seq(0), camera_name(camera_name), pix_fmt(pix_fmt)
	{
		std::ostringstream str;
		str << camera_name << "_link";
		link_name = str.str();

		epoch_offset = 0; // FIXME

		switch(pix_fmt) {
		case PIXEL_FORMAT_RAW8:
			// TODO need to define more informative RAW format, to also give bayer info, if available
			encoding = sensor_msgs::image_encodings::MONO8;
			bypp = 1;
			break;
		case PIXEL_FORMAT_RGB8:
			encoding = sensor_msgs::image_encodings::RGB8;
			bypp = 3;
			break;
		case PIXEL_FORMAT_BGR8:
			encoding = sensor_msgs::image_encodings::BGR8;
			bypp = 3;
			break;
		case PIXEL_FORMAT_UYV422:
			encoding = sensor_msgs::image_encodings::YUV422;
			bypp = 2;
			break;
		default:
			throw "Unsupported pixel format";
		}
	}

	void publish(const byteA& image, double timestamp) {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
		sensor_msgs::fillImage(msg, encoding, image.d0, image.d1, bypp * image.d1,
				image.p);
		msg.header.seq 		= seq++;
		msg.header.stamp	= ros::Time(timestamp + epoch_offset);
		msg.header.frame_id	= link_name;

		p.publish(msg);
#endif
	}
};

ImagePublisher::ImagePublisher(const std::string& topic, const std::string& camera_name, const PixelFormat pix_fmt) :
		s(new sImagePublisher(topic, camera_name, pix_fmt)) {

}

ImagePublisher::~ImagePublisher() {
	delete s;
}

void ImagePublisher::publish(const MT::Array<unsigned char>& image, double timestamp) {
	s->publish(image, timestamp);
}

void init_image_publishers(int argc, char* argv[], const char* name) {
#ifdef HAVE_ROS_IMAGE_TRANSPORT
	ros::init(argc, argv, name);
#endif
}

}
