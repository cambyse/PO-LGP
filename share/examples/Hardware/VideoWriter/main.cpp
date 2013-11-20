#include <Hardware/VideoWriter/videowriter.h>
#include <Core/array.h>

int main(int argc, char** argv){
  byteA img(200,100,3);
  img = 128;

  VideoWriter_x264 vid("z.kinectDepth.avi", img.d1, img.d0, 30, 20, "superfast");

  for(uint k=0;k<100;k++){
    vid.addFrame(img.p);
  }
  
  return 0;
}
