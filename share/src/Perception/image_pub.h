/*
 * image_pub.hpp
 *
 *  Created on: Jul 11, 2014
 *      Author: ingo
 */

#ifndef IMAGE_PUB_H_
#define IMAGE_PUB_H_

#include <string>
#include "pixel_format.h"

namespace MT {
template<class T> struct Array;
}
namespace MLR {
	struct sImagePublisher;

	class ImagePublisher {
	private:
		sImagePublisher *s;
	public:
		/** Create an image publisher for the give camera name and expecting that images have the specified pixel format. */
		ImagePublisher(const std::string& topic, const std::string& camera_name, const PixelFormat pix_fmt);
		~ImagePublisher();

		/** Publishes an image with the given capture timestamp. After this method returns, the content of "image" is
		 * no longer needed, even though publishing may occur in the background.
		 */
		void publish(const MT::Array<unsigned char>& image, double timestamp);
	};

	void init_image_publishers(int argc, char* argv[], const char* name);
}


#endif /* IMAGE_PUB_H_ */
