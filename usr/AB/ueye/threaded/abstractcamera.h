#ifndef ABSTRACTCAMERA_H
#define ABSTRACTCAMERA_H

#include <string>

class AbstractCamera {
  public:
    // open camera (can be issued from differen thread)
    virtual void open() = 0;

    // release camera buffers
    virtual void close() = 0;

    // grab frame (fast)
    virtual void grab() = 0;

    // decode frame
    virtual bool retrieve(char *img) = 0;

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getFPS() const { return fps; }
    std::string getName() const { return name; }

  protected:
    int width, height, fps;
    std::string name;
};

#endif // ABSTRACTCAMERA_H
