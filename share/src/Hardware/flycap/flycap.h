#pragma once

#include <sys/time.h>
#include <Core/module.h>
#include <exception>
#include <vector>
#include <Perception/pixel_format.h>

namespace MLR {

struct sFlycapInterface;

class FlycapInterface {
private:
	struct sFlycapInterface *s;
	bool streaming;
public:
	FlycapInterface(int cameraID, MLR::PixelFormat capture_format = MLR::PIXEL_FORMAT_BGR8, MLR::PixelFormat output_fmt = MLR::PIXEL_FORMAT_BGR8);
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
  struct MLR::sFlycapInterface *s;

  ACCESS(byteA, image);

  FlycapPoller();
  virtual ~FlycapPoller();

  void open();
  void step();
  void close();
};

