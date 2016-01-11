#pragma once

#include <sys/time.h>
#include <Core/module.h>
#include <exception>
#include <vector>
#include <Perception/pixel_format.h>

namespace mlr {

struct sFlycapInterface;

class FlycapInterface {
private:
	struct sFlycapInterface *s;
	bool streaming;
public:
	/**
	 * Creates flycap-based if for the given camera. If you don't know the camera ids,
	 * use the "flycap_serials" example, or call get_flycap_ids yourself.
	 *
	 * WARNING: Grabbing raw and asking for YUV as output DOES NOT WORK. It seems to be a missing feature in
	 * the flycapture library. If you grab RAW, either use the raw image directly, or convert to RGB/BGR.
	 */
	FlycapInterface(int cameraID, mlr::PixelFormat capture_format = mlr::PIXEL_FORMAT_RAW8, mlr::PixelFormat output_fmt = mlr::PIXEL_FORMAT_BGR8);
	~FlycapInterface();

	void startStreaming();
	bool grab(byteA& image, double& timestamp, unsigned int timeout=1<<31);
};
class FlycapException : public std::exception {
private:
	const std::string msg;

public:
	FlycapException(const std::string& msg) : msg(msg) {};
	virtual ~FlycapException() throw() {};
	const char* what() const throw() {
		return msg.c_str();
	}
};

std::vector<uint32_t> get_flycap_ids();

} // MLR

struct FlycapPoller: Module {
  struct mlr::sFlycapInterface *s;

  ACCESS(byteA, image);

  FlycapPoller();
  virtual ~FlycapPoller();

  void open();
  void step();
  void close();
};

