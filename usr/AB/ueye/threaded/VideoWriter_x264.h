#pragma once

#include <QMutex>

extern "C" {
#include<libavcodec/avcodec.h>
#include<libavformat/avformat.h>
#include<libswscale/swscale.h>
}

class VideoWriter_x264
{

public:

  // setup and open file
  VideoWriter_x264(const char *filename,
                    int width,
                    int height,
                    int frame_rate,
                    int crf,
                    const char *preset);

  // close file and clean-up
  ~VideoWriter_x264();

  // assuming width*height*3 bytes rgb interleaved data
  void addFrame(uint8_t *buffer);

private:

  // register lock manager and initialize libav
  void init();

  static int lockManagerQt(void **mutex, enum AVLockOp op);
  int writeEncodedFrame(AVPacket *pPacket);

  static bool initialized;
  static QMutex initMutex;

  AVFormatContext *out;
  AVCodecContext *enc;
  struct SwsContext *sws_ctx;
  AVFrame *pFrame;
  uint8_t *buffer;
  uint8_t *video_outbuf;
  int video_outbuf_size;
  unsigned int pts_step;
  int frames_in, frames_out;

};
