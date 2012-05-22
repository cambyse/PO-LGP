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

#include "hardware.h"
#include <perception/perception.h>

#ifdef MT_BUMBLE
  
#include <MT/vision.h>

#include <fstream>
#include <dc1394/dc1394.h>
#include "pgr_registers.h"
#include "pgr_stereocam.h"

struct Bumblebee;

//===========================================================================
//
// Camera process
//

struct DistPoly{
  doubleA           KL, KR, dL, dR, R, T, E, F;
  doubleA           RL, RR, PL, PR, Q;
  floatA            mapxL, mapyL;
  floatA            mapxR, mapyR;
};

struct CalibrationParameters{
  //output of cvStereoCalibrate
  arr cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
  arr R,T,E,F;
  //output of cvStereoRectify
  arr R1,R2,P1,P2,Q;
  //output of cvInitUndistortRectifyMap
  floatA map1L, map2L, map1R, map2R;

  void rectifyImages(byteA &imgL, byteA& imgR);
  void stereo2world(floatA& world,const floatA& stereo);
  void write(std::ostream& os) const;
  void read(std::istream& is);
};

struct Bumblebee {
  uint              width_, height_;
  bool              rectify_;
  DistPoly          distpoly_;

  bool has_parameters() {return (parameters_l.N>0 && parameters_r.N>0);};

  bool              is_initiated;
  bool              is_capturing;
  bool              is_adjusted;
  dc1394_t*         dc_context;
  doubleA           parameters_l, parameters_r;
  doubleA           mapd_l, mapd_r, mapu_l, mapu_r;
  PGRStereoCamera_t pgrstereocamera;
  byteA             buffer_raw, buffer_rgb, buffer_bw;
  byteA             buffer_rgbL, buffer_rgbR;

  Bumblebee();
  ~Bumblebee();
  void              init();
  void              init(uint64_t identifier);
  void              init(const char* identifier);
  void              deinit();
  void              start_capturing();
  void              stop_capturing();
  void              rectify(bool r);
  void              grab(byteA& rgb);
  void              grab(byteA& rgbL, byteA& rgbR);
  void              load_parameters(const char *filename);
  //void              distort(doubleA& xd, const doubleA& x, CamId c=RIGHT);
  //void              undistort(doubleA& xu, const doubleA& x, CamId c=RIGHT);
  bool              is_stereo();
};

struct sCamera{
  Image *camL,* camR;
  Bumblebee *cam;
  CalibrationParameters calib;
};

Camera::Camera():Process("BumblebeeCamera"){
  s = new sCamera;
  s->cam = NULL;
  birosInfo.getVariable(s->camL, "CameraL", this);
  birosInfo.getVariable(s->camR, "CameraR", this);
};

Camera::~Camera(){
  delete s;
}

void Camera::open(){
  s->cam = new Bumblebee();
  ifstream fil;
  MT::open(fil,"../../configurations/camera.cfg");
  s->calib.read(fil);
  step();
};

void Camera::step(){
  byteA tmpL,tmpR;
  s->cam->grab(tmpL, tmpR);
  //s->calib.rectifyImages(tmpL, tmpR);
  s->camR->set_img(tmpR, this);
  s->camL->set_img(tmpL, this);
};

void Camera::close(){
  s->cam->stop_capturing();
  s->cam->deinit();
  delete s->cam;
  s->cam = NULL;
};


//===========================================================================
//
// Bumblebee specifically
//

void dc1394_capture(Bumblebee& bb);

void connect(Bumblebee& bb, uint64_t identifier);
void* capture_helper(void*);


Bumblebee::Bumblebee() : is_initiated(false), is_capturing(false), is_adjusted(false){
  width_=1024; height_=768;
  dc_context = dc1394_new();
  // TODO this is bad: no hard-coding!!!!
  buffer_raw.resize(1536,1024,3); buffer_raw=0;
  buffer_rgb.resize(1536,1024,3); buffer_rgb=0;
  buffer_bw.resize(1536,1024); buffer_bw=0;;
  init();
};

Bumblebee::~Bumblebee()
{
  deinit();
};

void Bumblebee::init()
{init(49712223529894949ll);}

void Bumblebee::init(uint64_t identifier)
{
  if(is_initiated)
    return;
  connect(*this, identifier);
  is_initiated = true;
  if (!is_capturing)
    start_capturing();
  std::cout << "BB2: initiated" << std::endl;
};

void Bumblebee::init(const char* identifier) 
{init();};

void Bumblebee::deinit()
{
  if (!is_initiated)
    return;
  if (is_capturing)
    stop_capturing();

  // free camera's DC1394 resources
//   dc1394_camera_free(pgrstereocamera.camera);
  is_initiated = false;
  std::cout << "BB2: deinitiated (clean)" << std::endl;
};

void Bumblebee::start_capturing()
{
  if (is_capturing)
    return;

  int err = startTransmission(&pgrstereocamera);
  if (err != DC1394_SUCCESS)
    HALT("BB2: error after calling startTransmission()");

  // capture a couple of frame so the camera can warm up ... (adjust sensors)
  std::cout << "BB2: adjusting sensors " << std::flush;
  for (int fri = 0; fri < 25; fri++)
  {
    dc1394video_frame_t* frame = NULL;
    std::cout << "." << std::flush;
    dc1394_capture_dequeue(pgrstereocamera.camera, \
          DC1394_CAPTURE_POLICY_WAIT, &frame);
    dc1394_capture_enqueue(pgrstereocamera.camera, frame);
  }

  std::cout << " done." << std::endl;

  is_capturing = true;
  std::cout << "BB2: started capturing" << std::endl;
};

void Bumblebee::stop_capturing()
{
  if (is_capturing){
    is_capturing = false;          // this tells the capture thread to stop
    std::cout << "BB2: stopped thread" << std::endl;
  }

  // stop transmission
  dc1394_video_set_transmission(pgrstereocamera.camera, DC1394_OFF);
  dc1394_capture_stop(pgrstereocamera.camera);

  std::cout << "BB2: stopped capturing" << std::endl;
};

void Bumblebee::rectify(bool r){
  rectify_=r;
};

void Bumblebee::grab(byteA& rgb){
  dc1394_capture(*this);

  uint frame_length = (buffer_rgb.d0/2)*buffer_rgb.d1*3;

  if (rgb.d0 != buffer_rgb.d0/2 || rgb.d1 != buffer_rgb.d1 || rgb.nd != 3)
    rgb.resize(buffer_rgb.d0/2, buffer_rgb.d1, 3);

  // copy RGB frame
  for (uint i=0; i < frame_length; i++)
    rgb.p[i] = buffer_rgb.p[i];
};

void Bumblebee::grab(byteA& rgbL, byteA& rgbR){
  dc1394_capture(*this);
  
  uint frame_length = (buffer_rgb.d0/2)*buffer_rgb.d1*3;

  if(rgbL.d0 != buffer_rgb.d0/2 || rgbL.d1 != buffer_rgb.d1 || rgbL.nd != 3)
    rgbL.resize(buffer_rgb.d0/2, buffer_rgb.d1, 3);
  if(rgbR.d0 != buffer_rgb.d0/2 || rgbR.d1 != buffer_rgb.d1 || rgbR.nd != 3)
    rgbR.resize(buffer_rgb.d0/2, buffer_rgb.d1, 3);

  // copy RGB frame
  uint k=frame_length;
  for(uint i=0; i < frame_length; i++){
    rgbR.p[i] = buffer_rgb.p[i];
    rgbL.p[i] = buffer_rgb.p[k++];
  }
};

void Bumblebee::load_parameters(const char *filename)
{
  // load camera parameters from file
  std::ifstream is;
  is.open(filename);
  if (!is)
    HALT( "cannot open file");
  DistPoly &d=distpoly_;
  d.KL.readTagged(is,"KL");
  d.KR.readTagged(is,"KR");
  d.dL.readTagged(is,"dL");
  d.dR.readTagged(is,"dR");
  d.R.readTagged(is, "R");
  d.T.readTagged(is, "T");
  d.E.readTagged(is, "E");
  d.F.readTagged(is, "F");
  d.RL.readTagged(is,"RL");
  d.RR.readTagged(is,"RR");
  d.PL.readTagged(is,"PL");
  d.PR.readTagged(is,"PR");
  d.Q.readTagged(is, "Q");

  is.close();

  d.mapxL.resize(height_, width_);
  d.mapyL.resize(height_, width_);
  d.mapxR.resize(height_, width_);
  d.mapyR.resize(height_, width_);
  //cvInitUndistortRectifyMap(d.KL, d.dL, d.RL, d.PL, d.mapxL, d.mapyL);
  //cvInitUndistortRectifyMap(d.KR, d.dR, d.RR, d.PR, d.mapxR, d.mapyR);
//   // load camera parameters from file
//   std::ifstream is;
//   is.open(filename);
//   if (!is)
//     msg_error(HERE, "cannot open file");
// 
//   parameters_r.readTagged(is, "parameters_l_doubleA");
//   parameters_l.readTagged(is, "parameters_r_doubleA");
//   is.close();
// 
//   // distortion models for right and left camera, respectively
//   DistFov *distfov_r = new DistFov(
//                                    parameters_r(2),
//                                    parameters_r(3),
//                                    buffer_rgb.d1,
//                                    buffer_rgb.d0/2,
//                                    parameters_l(4)
//                                   );
//   DistFov *distfov_l = new DistFov(
//                                    parameters_l(2),
//                                    parameters_l(3),
//                                    buffer_rgb.d1,
//                                    buffer_rgb.d0/2,
//                                    parameters_l(4)
//                                   );
//   distfov_.append(distfov_r);
//   distfov_.append(distfov_l);
// 
// //  // initialize distortion and undistortion maps
// //  create_distortion_maps_fov(mapd_l,mapu_l,parameters_l);
// //  create_distortion_maps_fov(mapd_r,mapu_r,parameters_r);
}


bool Bumblebee::is_stereo()
{return true;};

void connect(Bumblebee& bb, uint64_t identifier)
{
  dc1394error_t err;

  // get a list of all devices on the ieee1394 bus
  dc1394camera_list_t* list = NULL;
  err = dc1394_camera_enumerate(bb.dc_context, &list);

  if (err != DC1394_SUCCESS)
    HALT( "BB2: did you 'chmod a+rw /dev/raw1394 /dev/video1394/0'?");
  if (list->num <= 0)
    HALT( "BB2: no cameras (check: plugged in? power supply?)");

  // use the first device on the bus, if no <identifier> has been specified
  // TODO

  // scan list for camera with specified guid
  dc1394camera_t* camera_dc = NULL;
  for (uint i = 0; i < list->num; i++)
  {
    if (identifier != list->ids[i].guid) continue;
    else
    {
      camera_dc = dc1394_camera_new(bb.dc_context, list->ids[i].guid);
      break;
    }
  }
  dc1394_camera_free_list(list);

  if (camera_dc == NULL)
    HALT( "BB2: could not find camera specified guid");

   // query camera information
   err = queryStereoCamera(camera_dc, &(bb.pgrstereocamera));
   if (err != DC1394_SUCCESS) HALT( "BB2: cannot query camera");

  // set video format etc.
//  err = setStereoVideoCapture(&bb.pgrstereocamera);
//  if (err != DC1394_SUCCESS)
//  {
//    std::cout <<"BB2: troubles setting video mode, retrying ... " << std::flush;

   err = dc1394_reset_bus(camera_dc);
   if (err != DC1394_SUCCESS) HALT( "BB2: reset failed");

   err = setStereoVideoCapture(&bb.pgrstereocamera);
   if (err != DC1394_SUCCESS) HALT( "BB2: reset failed");

//    std::cout << "done - all good!" << std::endl;
//  }
}

void dc1394_capture(Bumblebee& bb){

//   std::cout << "dc1394_capture" << std::endl;
  uint frame_length;
  dc1394video_frame_t* frame = NULL;
  dc1394error_t err = dc1394_capture_dequeue(bb.pgrstereocamera.camera, \
        DC1394_CAPTURE_POLICY_WAIT, &frame);
  if (err != DC1394_SUCCESS)
    HALT( "cannot capture frame");

  frame_length = bb.pgrstereocamera.nRows*bb.pgrstereocamera.nCols*6;

  // check temporary buffer size
  if (bb.buffer_raw.N != frame_length)
    bb.buffer_raw.resize(bb.pgrstereocamera.nRows*2, \
                          bb.pgrstereocamera.nCols,3);

  // deinterlace it (separate left and right)
  dc1394_deinterlace_stereo(
                            frame->image,
                            bb.buffer_raw.p,
                            bb.pgrstereocamera.nCols,
                            bb.pgrstereocamera.nRows*2
                            );

  // check buffer size and resize, if necessary
  if (bb.buffer_rgb.N != frame_length)
    bb.buffer_rgb.resize(bb.pgrstereocamera.nRows*2, \
                            bb.pgrstereocamera.nCols,3);

  // extract color from the bayer tile image
  // note: this will alias colors on the top and bottom rows
  err = dc1394_bayer_decoding_8bit(
                                    bb.buffer_raw.p,
                                    bb.buffer_rgb.p,
                                    bb.pgrstereocamera.nCols,
                                    bb.pgrstereocamera.nRows*2,
                                    bb.pgrstereocamera.bayerTile,
                                    DC1394_BAYER_METHOD_BILINEAR
                                  );
  if (err != DC1394_SUCCESS)
    HALT( "BB2: error during Bayer decoding");

  // return frame (memory) back to camera queue
  dc1394_capture_enqueue(bb.pgrstereocamera.camera, frame);
};



//===========================================================================
//
// CalibrationParameters
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

#else //MT_BUMBLE

struct sCamera{
  Image *camL,* camR;
  byteA dummyL, dummyR;
};

Camera::Camera():Process("BumblebeeCamera"){
  s = new sCamera;
  birosInfo.getVariable(s->camL, "CameraL", this);
  birosInfo.getVariable(s->camR, "CameraR", this);
  MT::String filename = birosInfo.getParameter<MT::String>("DummyCameraFile", this, STRING("DummyCameraImage_"));
  read_ppm(s->dummyL, STRING(filename <<"L.ppm"));
  read_ppm(s->dummyR, STRING(filename <<"R.ppm"));
};

Camera::~Camera(){
  delete s;
}

void Camera::open(){
  step();
};

void Camera::step(){
  s->camR->set_img(s->dummyR, this);
  s->camL->set_img(s->dummyL, this);
};

void Camera::close(){
};

#endif