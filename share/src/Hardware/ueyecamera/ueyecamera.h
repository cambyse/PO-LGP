#pragma once

#include<sys/time.h>
#include <Core/module.h>
#include <exception>

struct sUEyeInterface;

namespace MLR {
class UEyeInterface {
private:
	struct sUEyeInterface *s;
	bool streaming;
public:
	UEyeInterface(int cameraID);
	~UEyeInterface();

	void startStreaming();
	bool grab(byteA& image, double& timestamp, unsigned int timeout=1<<31);
};
class UEyeException : public std::exception {
private:
	const std::string msg;

public:
	UEyeException(const std::string& msg) : msg(msg) {};
	virtual ~UEyeException() throw() {};
	const char* what() const throw() {
		return msg.c_str();
	}
};

} // MLR

struct UEyePoller: Module {
  struct sUEyeInterface *s;

  ACCESS(byteA, ueye_rgb);
  //ACCESS(double, ueye_fps);

  UEyePoller();
  virtual ~UEyePoller();

  void open();
  void step();
  void close();
};

