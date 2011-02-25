
#include <imgproc.h>
#include <npors.h>
#include <cameracalibrator.h>
#include <calibrationimage.h>
#include <camera.h>
#include <opencv_helper.h>
#include <cvwindow.h>

int main(int argc,char **argv)
{
//#define TEST
#ifndef TEST
  byteA bwL, rgbL, bwR, rgbR;
//  np::Camera *cam = new np::Bumblebee2();
//  np::CameraCalibrator cc(cam, 9, 6, 0.0229, 0.8);
//  cam->load_parameters("camera.cfg");
  np::CameraCalibrator cc(NULL, 9, 6, 0.1, 0.6);
  cc.run();
//  cam->deinit();
//  delete cam;
#else
  uint h = 768;
  uint w = 1024;
  np::Camera *cam = new np::Bumblebee2();
  cam->load_parameters("camera.cfg");
  np::DistPoly &d = cam->distpoly_;

  np::CvWindow cvwindow("TEST");

  // init rectification maps
  floatA udmapxL(h,w), udmapyL(h,w), udmapxR(h,w), udmapyR(h,w);
  cvInitUndistortRectifyMap(d.KL, d.dL, d.RL, d.PL, udmapxL, udmapyL);
  cvInitUndistortRectifyMap(d.KR, d.dR, d.RR, d.PR, udmapxR, udmapyR);

  std::ostringstream caption;
  byteA bwL, bwR, rgbL, rgbR;
  byteA imgL_rect, imgR_rect, imgLR;
  bool r = true;
  for (uint i=0;;i++)
  {
    if (i%10==0)
      std::cout << "i = " << i << std::endl;
    // grab current left and right frames
    cam->grab(bwL, rgbL, bwR, rgbR);

//     // rectify images
//     if (r)
//     {
//       if (imgL_rect.N ==0) imgL_rect.resize(bwL.d0,bwL.d1,3);
//       if (imgR_rect.N ==0) imgR_rect.resize(bwR.d0,bwR.d1,3);
//       cvRemap(rgbL, imgL_rect, udmapxL, udmapyL);
//       cvRemap(rgbR, imgR_rect, udmapxR, udmapyR);
//     }
//     else
//     {
      np::array2array(imgL_rect, rgbL);
      np::array2array(imgR_rect, rgbR);
//     }

    if (i > 0 && i % 50 == 0)
    {
      r = !r;
      cam->rectify(r);
    }

    // merge left and right into one image
    np::merge(imgLR, imgL_rect, imgR_rect);

    // draw horizontal lines
    for (uint j = 50; j < imgLR.d0; j += 25)
      cvLine(imgLR, cvPoint(0,j), cvPoint(w*2,j), CV_RGB(0,255,0));

    // update caption
    caption.str("");
    caption << "ESC - quit, d - done";

    // display everything
    cvwindow.draw_image(imgLR);
    cvwindow.draw_caption(caption.str());
    cvwindow.update(0.8);

    // user interaction
    char c = cvWaitKey(10);
    // TODO outsource this into a separate function (command_handler)
    if (c == 27) // ESC - break this loop
      break;
    else if (c == 'd' || c == 'D')
      break;
  }
  cam->deinit();
  delete cam;
#endif
  return 0;
}
