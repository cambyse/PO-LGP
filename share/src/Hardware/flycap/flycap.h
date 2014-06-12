#pragma once

#include <sys/time.h>
#include <Core/module.h>
#include <exception>

struct sFlycapInterface;

namespace MLR {
class FlycapInterface {
private:
	struct sFlycapInterface *s;
	bool streaming;
public:
	FlycapInterface(int cameraID);
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

} // MLR

struct FlycapPoller: Module {
  struct sFlycapInterface *s;

  ACCESS(byteA, rgb);

  FlycapPoller();
  virtual ~FlycapPoller();

  void open();
  void step();
  void close();
};

