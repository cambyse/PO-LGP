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

#ifdef MT_OPENCV
#undef COUNT
#include <opencv/highgui.h>
#include <opencv/cv.h>
#undef MIN
#undef MAX
#endif

#include <MT/vision.h>

#include "camera.h"
#include <hardware/uvccamera.h>




struct sCameraModule{
#ifdef MT_BUMBLE
  np::Bumblebee2 *cam;
#else
  void *cam;
#endif
};


#ifdef MT_BUMBLE
CameraModule::CameraModule():Process("BumblebeeProcess"){
  s = new sCameraModule;
};

void CameraModule::open(){
  s->cam = new np::Bumblebee2();
  ifstream fil;
  MT::open(fil,"../../configurations/camera.cfg");
  calib.read(fil);
  step();
};

void CameraModule::step(){
  byteA tmpL,tmpR;
  s->cam->grab(tmpL, tmpR);
  calib.rectifyImages(tmpL, tmpR);
  if(output){
    output->writeAccess(this);
    output->rgbL=tmpL;
    output->rgbR=tmpR;
    output->deAccess(this);
  }else{
    MT_MSG("Warning: camera writes into /dev/null");
  }
};

void CameraModule::close(){
  s->cam->stop_capturing();
  s->cam->deinit();
  delete s->cam;
};
#else
CameraModule::CameraModule():Process("BumblebeeProcess"){}
void CameraModule::open(){ MT_MSG("Warning: opening dummy Camera class"); }
void CameraModule::step(){ }
void CameraModule::close(){ MT_MSG("Warning: closing dummy Camera class"); }
#endif

//void np::create_distortion_maps_fov(
//                                    doubleA& mapd,
//                                    doubleA& mapu,
//                                    const doubleA& parameters,
//                                    uint width,
//                                    uint height
//                                   )
//{
//  double widthinv=1./width, heightinv=1./height;
//  double cx=parameters(2), cy=parameters(3);
//  double ru,rd,w=parameters(4);
//  double two_tanw2 = 2.*tan(w/2.), winv=1./w;
//  double two_tanw2inv = 1./two_tanw2;
//  mapd.resize(height,width);
//  mapu.resize(height,width);

//  // distortion map
//  double xt, yt;
//  for (uint y=0;y<height;y++)
//  {
//    for (uint x=0;x<width;x++)
//    {
//      xt = (x*widthinv)-cx;               // normalize and center data
//      yt = (y*heightinv)-cy;
//      ru = sqrt(xt*xt+yt*yt);             // distance to principal point
//      rd = winv*atan(ru*two_tanw2)/ru;    // distortion function
//      mapd(y,x) = (ru < 0.001 ? 1. : rd/ru);

//      ru = tan(ru*w)*two_tanw2inv;   // undistortion function
//      mapu(y,x) = (rd < 0.001 ? 1. : ru/rd);
//    }
//  }

////r = arrayfun(@(ru)(winv*atan(ru*two_tanw2)), ru)./ru;
////%mapd = reshape(r,height,width);
////Xdn = [Xu(:,1).*r Xu(:,2).*r];
////Xd = [Xu(:,1).*r*N(1)+C(1) Xu(:,2).*r*N(2)+C(2)];
//  // C Square Sum Inverted
////   double widthinv=1./(double)width, heightinv=1./(double)height;
////  double cs=sqrt(cx*cx+cy*cy);
////  double csinv=1./cs;
////  double xt, yt;
////  for (uint y=0;y<height;y++)
////  {
////    for (uint x=0;x<width;x++)
////    {
////      // distortion map
////      xt = x - cx;
////      yt = y - cy;
////      ru = sqrt(xt*xt+yt*yt);                   // distance from principal point
////      rd = winv*atan(ru*csinv*w2tan);           // distortion function
////      mapd(y,x) = (ru < 0.001 ? 1. : rd/ru);

////      // undistortion map
////      ru = tan(ru*w)/w2tan;   // undistortion function
////      mapu(y,x) = (rd < 0.001 ? 1. : ru/rd);
////    }
////  }
////  std::ofstream os;
////  os.open("mapu.temp");
////  mapu.writeTagged(os,"xu2",false);
////  os.close();

////   %% distort map
//// Cs=sqrt(sum(C.^2));
//// Csinv=1/Cs;
//// XU=[X(:)-C(1), Y(:)-C(2)];                  % center undistorted around prin. p.
//// RU = sqrt(XU(:,1).^2 + XU(:,2).^2);     % distance from principal point %%% /sqrt(sum(C.^2))
//// RD = (1/w) * atan(2*RU*Csinv*tan(w/2));           % distortion function
//// R1 = (RD./RU);                              % distortion factor
//// R1(isnan(R1(:)))=1;
//// mapd = reshape(R1,height,width);
//// 
//// %% undistort map
//// RU = tan(RD*w)/(2*tan(w/2));                % undistortion function
//// R2 = (RU./RD);                              % undistortion factor
//// R2(isnan(R2(:)))=1;
//// mapu = reshape(R2,height,width);
//// clear RU RD R1 R2;

//}


//===========================================================================
//
// CAMERA CALIBRATION
//

void CalibrationParameters::read(std::istream& is){
  cameraMatrix1.readTagged(is, "cameraMatrix1");
  cameraMatrix2.readTagged(is, "cameraMatrix2");
  distCoeffs1.readTagged(is, "distCoeffs1");
  distCoeffs2.readTagged(is, "distCoeffs2");
  R.readTagged(is,  "R");
  T.readTagged(is,  "T");
  E.readTagged(is,  "E");
  F.readTagged(is,  "F");
  R1.readTagged(is, "R1");
  R2.readTagged(is, "R2");
  P1.readTagged(is, "P1");
  P2.readTagged(is, "P2");
  Q.readTagged(is,  "Q");
}

void CalibrationParameters::write(std::ostream& os) const{
  cameraMatrix1.writeTagged(os, "cameraMatrix1", false); os << std::endl;
  cameraMatrix2.writeTagged(os, "cameraMatrix2", false); os << std::endl;
  distCoeffs1.writeTagged(os, "distCoeffs1", false); os << std::endl;
  distCoeffs2.writeTagged(os, "distCoeffs2", false); os << std::endl;
  R.writeTagged(os,  "R", false);  os << std::endl;
  T.writeTagged(os,  "T", false);  os << std::endl;
  E.writeTagged(os,  "E", false);  os << std::endl;
  F.writeTagged(os,  "F", false);  os << std::endl;
  R1.writeTagged(os, "R1", false); os << std::endl;
  R2.writeTagged(os, "R2", false); os << std::endl;
  P1.writeTagged(os, "P1", false); os << std::endl;
  P2.writeTagged(os, "P2", false); os << std::endl;
  Q.writeTagged(os,  "Q", false);
}

void CalibrationParameters::rectifyImages(byteA &imgL, byteA& imgR){
#ifdef MT_OPENCV
  CvMatDonor cvMatDonor;
  if(!map1L.N){
    map1L.resize(imgL.d0,imgL.d1);
    map2L.resize(imgL.d0,imgL.d1);
    map1R.resize(imgL.d0,imgL.d1);
    map2R.resize(imgL.d0,imgL.d1);
    cvInitUndistortRectifyMap(CVMAT(cameraMatrix1), CVMAT(distCoeffs1),
			      CVMAT(R1), CVMAT(P1),
			      CVMAT(map1L), CVMAT(map2L));
    cvInitUndistortRectifyMap(CVMAT(cameraMatrix2), CVMAT(distCoeffs2),
			      CVMAT(R2), CVMAT(P2),
			      CVMAT(map1R), CVMAT(map2R));
  }
  byteA tmp=imgL;
  cvRemap(CVMAT(tmp), CVMAT(imgL), CVMAT(map1L), CVMAT(map2L));
  tmp=imgR;
  cvRemap(CVMAT(tmp), CVMAT(imgR), CVMAT(map1R), CVMAT(map2R));
#else
  HALT("Can't call this function without opencv");
#endif
}

void CalibrationParameters::stereo2world(floatA& world,const floatA& stereo){
#ifdef MT_OPENCV
  CvMatDonor cvMatDonor;
  cvPerspectiveTransform(CVMAT(stereo), CVMAT(world), CVMAT(Q));
#else
  HALT("Can't call this function without opencv");
#endif
}

