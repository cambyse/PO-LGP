/*  Copyright 2010 Nils Plath
    email: nilsp@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

#include <sstream>
#include "cameracalibrator.h"
#include "calibrationimage.h"
#include "camera.h"
#include "nputils.h"
#include "cvutils.h"
#include "cvwindow.h"
#include "opencv_helper.h"
#include <MT/vision.h>


np::CameraCalibrator::CameraCalibrator(Camera* c, uint x, uint y, double l, double scale)
  : scale_(scale), camera_(c), edge_(l)
{
  size_[0]=x; size_[1]=y; 
  cvwindow_ = new CvWindow("CameraCalibrator");
};

np::CameraCalibrator::~CameraCalibrator()
{
  delete cvwindow_;
};

void np::CameraCalibrator::run()
{
  if (detect()==-1) return;
  if (display()==-1) return;
  if (calibrate()==-1) return;
  if (rectify()==-1) return;
  display_result();
};

void np::CameraCalibrator::reset()
{
  calibration_images_.clear();
  std::cout << "reset" << std::endl;
};

int np::CameraCalibrator::detect()
{
  bool added_frames=false;
  std::ostringstream caption, f;
  byteA bwL, bwR, bwL2, bwR2, rgbL, rgbR, bwLR;
  for (uint i=0;;i++)
  {
// #define DEBUG_CAMCALIB
#if defined(DEBUG_CAMCALIB)
    static int count =1;
    if(count >= 15)
      break;
    if (count == 10)
      count++;

    f.str("");
    f << "/home/nils/uni/code/3rdparty/opencv/svn-repo/opencv/samples/c/left";
    f << std::setw(2) << std::setfill('0') << count << ".jpg";
    load_image(bwL, f.str().c_str(), 0);
    CalibrationImage *ciL = new CalibrationImage(bwL, size_[0], size_[1], edge_);
    f.str("");
    f << "/home/nils/uni/code/3rdparty/opencv/svn-repo/opencv/samples/c/right";
    f << std::setw(2) << std::setfill('0') << count << ".jpg";
    load_image(bwR, f.str().c_str(), 0);
    CalibrationImage *ciR = new CalibrationImage(bwR, size_[0], size_[1], edge_);
    count++;
    if (bwL2.N==0) bwL2.resize(bwL.d0, bwL.d1, 3);
    cvCvtColor(bwL2, bwL, CV_GRAY2RGB);
    if (bwR2.N==0) bwR2.resize(bwR.d0, bwR.d1, 3);
    cvCvtColor(bwR2, bwR, CV_GRAY2RGB);
    if (ciL->found_pattern() && ciR->found_pattern())
      {ciL->draw(bwL2); ciR->draw(bwR2);}
    if (ciL->found_pattern() && ciR->found_pattern())
    {
      calibration_images_.append(ciL); calibration_images_.append(ciR);
      added_frames=true;
    }
#else
    // grab current left and right frames
    camera_->grab(bwL, rgbL, bwR, rgbR);

    // detect chessboards
    CalibrationImage *ciL = new CalibrationImage(bwL, size_[0], size_[1], edge_);
    CalibrationImage *ciR = new CalibrationImage(bwR, size_[0], size_[1], edge_);
    if (bwL2.N==0) bwL2.resize(bwL.d0, bwL.d1, 3);
    cvCvtColor(bwL2, bwL, CV_GRAY2RGB);
    if (bwR2.N==0) bwR2.resize(bwR.d0, bwR.d1, 3);
    cvCvtColor(bwR2, bwR, CV_GRAY2RGB);
    if (ciL->found_pattern() && ciR->found_pattern())
      {ciL->draw(bwL2); ciR->draw(bwR2);}

    // update caption
    caption.str("");
    caption << "ESC - quit, r - reset, g - grab, d - done";
    caption << "                                                   ";
    caption << "calibration images: " << std::setw(3) << (camera_->is_stereo() ?
         calibration_images_.d0/2 : calibration_images_.d0)
      << (camera_->is_stereo() ? "x2" : " ");
#endif


    // display everything
    merge(bwLR, bwL2, bwR2);
    cvwindow_->draw_image(bwLR);
    cvwindow_->draw_caption(caption.str());
    cvwindow_->update(scale_);

    // user interaction
    char c = cvWaitKey(10);
    // TODO outsource this into a separate function (command_handler)
    if (c == 27) // ESC - break this loop
      return -1;
    else if (c == 'r' || c == 'R')
      reset();
    else if (c == 'd' || c == 'D')
      return 0;
    else if (c == 'g' || c == 'G')
    {
      if (ciL->found_pattern() && ciR->found_pattern())
      {
        calibration_images_.append(ciL); calibration_images_.append(ciR);
        added_frames=true;
      }
    }

    if (!added_frames)
    {
      delete ciL;
      delete ciR;
    }
    else
      added_frames=false;
  }
  return 0;
};

int np::CameraCalibrator::display()
{
  std::ostringstream caption;
  uint idx=0;
  char c=1;
#ifndef DEBUG_CAMCALIB
  if (camera_->is_stereo())
    calibration_images_.reshape(calibration_images_.d0/2,2);
#endif

  CalibrationImage *ciL, *ciR;
  byteA imgL, imgR, imgLR;
  for (;;)
  {
    // get current calibration image
    ciL = calibration_images_(idx,0);
    ciR = calibration_images_(idx,1);
    if (imgL.N==0) imgL.resize(ciL->image_.d0, ciL->image_.d1, 3);
    cvCvtColor(imgL, ciL->image_, CV_GRAY2RGB);
    if (imgR.N==0) imgR.resize(ciR->image_.d0, ciR->image_.d1, 3);
    cvCvtColor(imgR, ciR->image_, CV_GRAY2RGB);

    // draw chessboard pattern
    ciL->draw(imgL);
    ciR->draw(imgR);

    // update caption
    caption.str("");
    caption << "ESC - quit, r - remove, n - next, p - previous, d - done";
    caption << "                                   ";
    caption << "calibration images: " << std::setw(3) << (idx+1) << "/";
    caption << std::setw(3) << calibration_images_.d0;

    // display everything
    merge(imgLR, imgL, imgR);
    cvwindow_->draw_image(imgLR);
    cvwindow_->draw_caption(caption.str());
    cvwindow_->update(scale_);

    c = cvWaitKey(0);
    if (c == 27) // ESC - break this loop
      return -1;
    else if (c == 'r' || c == 'R')
    {
      calibration_images_.delRow(idx);
      if (calibration_images_.d0 == 0)
        return -1;
      if (idx >= calibration_images_.d0)
        idx--;
    }
    else if (c == 'p' || c == 'P')
      idx = (idx>0 ? idx-1 : 0);
    else if (c == 'n' || c == 'N')
      idx = (idx<calibration_images_.d0-1 ? idx+1 : calibration_images_.d0-1);
    else if (c == 'd' || c == 'D')
      return 0;
  };
};


void get_pts3d(MT::Array<CvPoint3D32f> &pts3d, uint num_calibimg, uint x, uint y, double l)
{
  uint num_squares = x*y;
  MT::Array<CvPoint3D32f> pts3d_temp(num_squares);
  for (uint i = 0; i < x; i++)
    for (uint j = 0; j < y; j++)
    {
      pts3d_temp.p[j*x + i].x = i*l;
      pts3d_temp.p[j*x + i].y = j*l;
      pts3d_temp.p[j*x + i].z = 0.;
    }

  // ... for each calibration image
  for (uint i = 0; i < num_calibimg; i++)
    pts3d.append(pts3d_temp);
};

void get_pts2d(
               MT::Array<CvPoint2D32f> &pts2dL,
               MT::Array<CvPoint2D32f> &pts2dR,
               MT::Array<np::CalibrationImage*> ci,
               uint num_squares
              )
{
  for (uint i = 0; i < ci.d0; i++)
  {
    std::vector<CvPoint2D32f> &cornersL = ci(i,0)->corners2_;
    std::vector<CvPoint2D32f> &cornersR = ci(i,1)->corners2_;
    if (cornersL.size() != cornersR.size() || cornersL.size() != num_squares)
    {
      std::cout << "cornersL.size() = " << cornersL.size() << " ";
      std::cout << "cornersR.size() = " << cornersR.size() << " ";
      std::cout << "num_squares     = " << num_squares << std::endl;
      np::msg_error(HERE, "number of 2D points in left/right image are not correct");
    }
    for (uint j = 0; j < cornersL.size(); j++)
    {
      pts2dL.append(cornersL[j]);
      pts2dR.append(cornersR[j]);
    }
  }
};

inline double reprojection_error
(
  doubleA &KL,
  doubleA& KR,
  doubleA& dL,
  doubleA &dR,
  doubleA& F,
  MT::Array<CvPoint2D32f> &pts2dL,
  MT::Array<CvPoint2D32f> &pts2dR
)
{
  double avg_err = 0., err = 0.;
  uint N = pts2dL.d0;
  MT::Array<CvPoint3D32f> lines[2];
  lines[0].resize(N);
  lines[1].resize(N);
  CvMat cvlinesL = cvMat(1, N, CV_32FC3, lines[0].p);
  CvMat cvlinesR = cvMat(1, N, CV_32FC3, lines[1].p);

  CvMat cvpts2dL = cvMat(1, N, CV_32FC2, pts2dL.p);
  CvMat cvpts2dR = cvMat(1, N, CV_32FC2, pts2dR.p);
  CvMat cvKL = cvMat(3, 3, CV_64F, KL.p );
  CvMat cvKR = cvMat(3, 3, CV_64F, KR.p );
  CvMat cvdL = cvMat(1, 5, CV_64F, dL.p );
  CvMat cvdR = cvMat(1, 5, CV_64F, dR.p );
  CvMat cvF = cvMat(3, 3, CV_64F, F.p );

  // undistort points
  // TODO wrap these in opencv_helper.h
  cvUndistortPoints(&cvpts2dL, &cvpts2dL, &cvKL, &cvdL, 0, &cvKL);
  cvUndistortPoints(&cvpts2dR, &cvpts2dR, &cvKR, &cvdR, 0, &cvKR);
  cvComputeCorrespondEpilines(&cvpts2dL, 1, &cvF, &cvlinesL);
  cvComputeCorrespondEpilines(&cvpts2dR, 2, &cvF, &cvlinesR);

  for(uint i = 0; i < N; i++)
  {
    err = 
     fabs(pts2dL(i).x*lines[1](i).x + pts2dL(i).y*lines[1](i).y + lines[1](i).z)
    +fabs(pts2dR(i).x*lines[0](i).x + pts2dR(i).y*lines[0](i).y + lines[0](i).z);
     avg_err += err;
  }

  return avg_err/N;
};

double np::CameraCalibrator::calibrate()
{
  if (calibration_images_.N==0)
    return -1;

  calibration_images_.reshape(calibration_images_.N/2,2);

  // set up 3D coordinates of chessboard corners ...
  uint height = calibration_images_.elem(0)->image_.d0;
  uint width = calibration_images_.elem(0)->image_.d1;
  uint num_calibimg = calibration_images_.d0;
  uint num_squares = (size_[0])*(size_[1]);
  MT::Array<CvPoint3D32f> pts3d;
  std::cout << "edge_ = " << edge_ << std::endl;
  get_pts3d(pts3d, num_calibimg, size_[0], size_[1], edge_);

  // collect all 2D points
  MT::Array<CvPoint2D32f> pts2dL, pts2dR;
  get_pts2d(pts2dL, pts2dR, calibration_images_, num_squares);
  intA num_pts(num_calibimg); num_pts = num_squares;

  // stereo calibration & re-projection error
  double rpe;
  rpe = cvStereoCalibrate(
                          pts3d,
                          pts2dL,
                          pts2dR,
                          (num_pts),
                          (cameraMatrix1), (distCoeffs1), (cameraMatrix2), (distCoeffs2),
                          cvSize(width, height),
                          (R), (T), (E), (F)
                         );

#if defined(NP_DEBUG)
  std::cout << "KL  = " << std::endl << cameraMatrix1 << std::endl;
  std::cout << "KR  = " << std::endl << cameraMatrix2 << std::endl;
  std::cout << "dL  = " << std::endl << distCoeffs1 << std::endl;
  std::cout << "dR  = " << std::endl << distCoeffs2 << std::endl;
  std::cout << "R   = " << std::endl << R << std::endl;
  std::cout << "T   = " << std::endl << T << std::endl;
  std::cout << "E   = " << std::endl << E << std::endl;
  std::cout << "F   = " << std::endl << F << std::endl;
#endif

  std::cout << "avg error = " << rpe/pts2dL.d0 << " ";
  rpe = reprojection_error(cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, F, pts2dL, pts2dR);
  std::cout << "avg epipolar error = " << rpe << std::endl;
  return rpe;
};

int np::CameraCalibrator::rectify()
{
  uint h = calibration_images_.elem(0)->image_.d0;
  uint w = calibration_images_.elem(0)->image_.d1;
  cvStereoRectify(cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, cvSize(w,h), R, T, R1, R2, P1, P2, Q);
#if defined(NP_DEBUG)
  std::cout << "RL  = " << std::endl << R1 << std::endl;
  std::cout << "RR  = " << std::endl << R2 << std::endl;
  std::cout << "PL  = " << std::endl << P1 << std::endl;
  std::cout << "PR  = " << std::endl << P2 << std::endl;
  std::cout << "Q   = " << std::endl << Q  << std::endl;
#endif
  return 0;
}

int np::CameraCalibrator::display_result()
{
  
  uint h = calibration_images_.elem(0)->image_.d0;
  uint w = calibration_images_.elem(0)->image_.d1;

  // init rectification maps
  map1L.resize(h,w);
  map2L.resize(h,w);
  map1R.resize(h,w);
  map2R.resize(h,w);
  cvInitUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, map1L, map2L);
  cvInitUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, map1R, map2R);

  std::ostringstream caption;
  byteA bwL, bwR, rgbL, rgbR;
  byteA imgL_rect, imgR_rect, imgLR;
#ifndef DEBUG_CAMCALIB
  for (uint i=0;;i++)
  {
    // grab current left and right frames
    camera_->grab(bwL, rgbL, bwR, rgbR);

    // rectify images
    if (imgL_rect.N ==0) imgL_rect.resize(bwL.d0,bwL.d1,3);
    if (imgR_rect.N ==0) imgR_rect.resize(bwR.d0,bwR.d1,3);
    cvRemap(rgbL, imgL_rect, map1L, map2L);
    cvRemap(rgbR, imgR_rect, map1R, map2R);

    // merge left and right into one image
    merge(imgLR, imgL_rect, imgR_rect);

    // draw horizontal lines
    for (uint j = 50; j < imgLR.d0; j += 25)
      cvLine(imgLR, cvPoint(0,j), cvPoint(w*2,j), CV_RGB(0,255,0));

    // update caption
    caption.str("");
    caption << "ESC - quit, d - done";

    // display everything
    cvwindow_->draw_image(imgLR);
    cvwindow_->draw_caption(caption.str());
    cvwindow_->update(scale_);

    // user interaction
    char c = cvWaitKey(10);
    // TODO outsource this into a separate function (command_handler)
    if (c == 27) // ESC - break this loop
      return -1;
    else if (c == 'd' || c == 'D')
      return 0;
    else if (c == 's' || c == 'S'){
      ofstream fil("camera.cfg");
      write(fil);
    }
  }
#endif

  return 0;
};


void np::CameraCalibrator::command_handler(const char *cmd)
{msg_missing_implementation(HERE,"np::CameraCalibrator::command_handler()");};

