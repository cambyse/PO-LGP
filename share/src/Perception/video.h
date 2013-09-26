/* minimalistic wrapper of the standard libav (ffmpeg) example for
   encoding/decoding vids */

#include <Core/array.h>

struct VideoEncoder{
  struct sVideoEncoder *s;
  VideoEncoder(const char* filename="z.avi");
  void addFrame(const byteA& rgb);
  void close();
};
