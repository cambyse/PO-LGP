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

#include "camera.h"
#include "nputils.h"
#include "cvutils.h"
//#include "opencv_helper.h"

#if defined(NP_DC1394)
#include <fstream>
#include <dc1394/dc1394.h>

#include "pgr_registers.h"
#include "pgr_stereocam.h"

#endif

#if defined(NP_DC1394)
namespace np
{
void dc1394_capture(Bumblebee2WS& ws);

struct Bumblebee2WS
{
  Bumblebee2WS() : is_initiated(false), is_capturing(false), is_adjusted(false)
  {
    dc_context = dc1394_new();
    // TODO this is bad: no hard-coding!!!!
    buffer_raw.resize(1536,1024,3); buffer_raw=0;
    buffer_rgb.resize(1536,1024,3); buffer_rgb=0;
    buffer_bw.resize(1536,1024); buffer_bw=0;;
  };
  ~Bumblebee2WS(){};
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
};
void connect(Bumblebee2WS& ws, uint64_t identifier);
void* capture_helper(void*);
}; // namespace np

np::Bumblebee2::Bumblebee2(){
  width_=1024; height_=768;
  ws_ = new Bumblebee2WS();
  init();
};

np::Bumblebee2::~Bumblebee2()
{
  deinit();
  delete ws_;
};

void np::Bumblebee2::init()
{init(49712223529894949ll);}

void np::Bumblebee2::init(uint64_t identifier)
{
  if(ws_->is_initiated)
    return;
  connect(*ws_, identifier);
  ws_->is_initiated = true;
  if (!ws_->is_capturing)
    start_capturing();
  std::cout << "BB2: initiated" << std::endl;
};

void np::Bumblebee2::init(const char* identifier) 
{init();};

void np::Bumblebee2::deinit()
{
  if (!ws_->is_initiated)
    return;
  if (ws_->is_capturing)
    stop_capturing();

  // free camera's DC1394 resources
//   dc1394_camera_free(ws_->pgrstereocamera.camera);
  ws_->is_initiated = false;
  std::cout << "BB2: deinitiated (clean)" << std::endl;
};

void np::Bumblebee2::start_capturing()
{
  if (ws_->is_capturing)
    return;

  int err = startTransmission(&ws_->pgrstereocamera);
  if (err != DC1394_SUCCESS)
    msg_error(HERE,"BB2: error after calling startTransmission()");

  // capture a couple of frame so the camera can warm up ... (adjust sensors)
  std::cout << "BB2: adjusting sensors " << std::flush;
  for (int fri = 0; fri < 25; fri++)
  {
    dc1394video_frame_t* frame = NULL;
    std::cout << "." << std::flush;
    dc1394_capture_dequeue(ws_->pgrstereocamera.camera, \
          DC1394_CAPTURE_POLICY_WAIT, &frame);
    dc1394_capture_enqueue(ws_->pgrstereocamera.camera, frame);
  }

  std::cout << " done." << std::endl;

  ws_->is_capturing = true;
  std::cout << "BB2: started capturing" << std::endl;
};

void np::Bumblebee2::stop_capturing()
{
  if (ws_->is_capturing){
    ws_->is_capturing = false;          // this tells the capture thread to stop
    std::cout << "BB2: stopped thread" << std::endl;
  }

  // stop transmission
  dc1394_video_set_transmission(ws_->pgrstereocamera.camera, DC1394_OFF);
  dc1394_capture_stop(ws_->pgrstereocamera.camera);

  std::cout << "BB2: stopped capturing" << std::endl;
};

void np::Bumblebee2::rectify(bool r){
  rectify_=r;
};

void np::Bumblebee2::grab(byteA& rgb){
  dc1394_capture(*ws_);

  uint frame_length = (ws_->buffer_rgb.d0/2)*ws_->buffer_rgb.d1*3;

  if (rgb.d0 != ws_->buffer_rgb.d0/2 || rgb.d1 != ws_->buffer_rgb.d1 || rgb.nd != 3)
    rgb.resize(ws_->buffer_rgb.d0/2, ws_->buffer_rgb.d1, 3);

  // copy RGB frame
  for (uint i=0; i < frame_length; i++)
    rgb.p[i] = ws_->buffer_rgb.p[i];
};

void np::Bumblebee2::grab(byteA& rgbL, byteA& rgbR){
  dc1394_capture(*ws_);
  
  uint frame_length = (ws_->buffer_rgb.d0/2)*ws_->buffer_rgb.d1*3;

  if(rgbL.d0 != ws_->buffer_rgb.d0/2 || rgbL.d1 != ws_->buffer_rgb.d1 || rgbL.nd != 3)
    rgbL.resize(ws_->buffer_rgb.d0/2, ws_->buffer_rgb.d1, 3);
  if(rgbR.d0 != ws_->buffer_rgb.d0/2 || rgbR.d1 != ws_->buffer_rgb.d1 || rgbR.nd != 3)
    rgbR.resize(ws_->buffer_rgb.d0/2, ws_->buffer_rgb.d1, 3);

  // copy RGB frame
  uint k=frame_length;
  for(uint i=0; i < frame_length; i++){
    rgbR.p[i] = ws_->buffer_rgb.p[i];
    rgbL.p[i] = ws_->buffer_rgb.p[k++];
  }
};

void np::Bumblebee2::load_parameters(const char *filename)
{
  // load camera parameters from file
  std::ifstream is;
  is.open(filename);
  if (!is)
    msg_error(HERE, "cannot open file");
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
//   ws_->parameters_r.readTagged(is, "parameters_l_doubleA");
//   ws_->parameters_l.readTagged(is, "parameters_r_doubleA");
//   is.close();
// 
//   // distortion models for right and left camera, respectively
//   DistFov *distfov_r = new DistFov(
//                                    ws_->parameters_r(2),
//                                    ws_->parameters_r(3),
//                                    ws_->buffer_rgb.d1,
//                                    ws_->buffer_rgb.d0/2,
//                                    ws_->parameters_l(4)
//                                   );
//   DistFov *distfov_l = new DistFov(
//                                    ws_->parameters_l(2),
//                                    ws_->parameters_l(3),
//                                    ws_->buffer_rgb.d1,
//                                    ws_->buffer_rgb.d0/2,
//                                    ws_->parameters_l(4)
//                                   );
//   distfov_.append(distfov_r);
//   distfov_.append(distfov_l);
// 
// //  // initialize distortion and undistortion maps
// //  create_distortion_maps_fov(ws_->mapd_l,ws_->mapu_l,ws_->parameters_l);
// //  create_distortion_maps_fov(ws_->mapd_r,ws_->mapu_r,ws_->parameters_r);
}

void np::Bumblebee2::undistort(doubleA& xu, const doubleA& xd, CamId c)
{
// TODO
//   if (!ws_->has_parameters() || !distfov_.N == 2)
//   {
//     array2array(xu,xd);
//     msg_error(HERE, "camera parameters have not been loaded!", false);
//   }
// 
//   DistFov *distfov = (c==RIGHT ? this->distfov_(0) : this->distfov_(1));
//   distfov->undistort(xu,xd);
};

void np::Bumblebee2::distort(doubleA& xd, const doubleA& xu, CamId c)
{
// TODO
//   if (!ws_->has_parameters() || !distfov_.N == 2)
//   {
//     array2array(xd,xu);
//     msg_error(HERE, "camera parameters have not been loaded!", false);
//   }
// 
//   DistFov *distfov = (c==RIGHT ? this->distfov_(0) : this->distfov_(1));
//   distfov->distort(xd,xu);
};

bool np::Bumblebee2::is_stereo()
{return true;};

void np::connect(Bumblebee2WS& ws, uint64_t identifier)
{
  dc1394error_t err;

  // get a list of all devices on the ieee1394 bus
  dc1394camera_list_t* list = NULL;
  err = dc1394_camera_enumerate(ws.dc_context, &list);

  if (err != DC1394_SUCCESS)
    msg_error(HERE, "BB2: did you 'chmod a+rw /dev/raw1394 /dev/video1394/0'?");
  if (list->num <= 0)
    msg_error(HERE, "BB2: no cameras (check: plugged in? power supply?)");

  // use the first device on the bus, if no <identifier> has been specified
  // TODO

  // scan list for camera with specified guid
  dc1394camera_t* camera_dc = NULL;
  for (uint i = 0; i < list->num; i++)
  {
    if (identifier != list->ids[i].guid) continue;
    else
    {
      camera_dc = dc1394_camera_new(ws.dc_context, list->ids[i].guid);
      break;
    }
  }
  dc1394_camera_free_list(list);

  if (camera_dc == NULL)
    msg_error(HERE, "BB2: could not find camera specified guid");

   // query camera information
   err = queryStereoCamera(camera_dc, &(ws.pgrstereocamera));
   if (err != DC1394_SUCCESS) msg_error(HERE, "BB2: cannot query camera");

  // set video format etc.
//  err = setStereoVideoCapture(&ws.pgrstereocamera);
//  if (err != DC1394_SUCCESS)
//  {
//    std::cout <<"BB2: troubles setting video mode, retrying ... " << std::flush;

   err = dc1394_reset_bus(camera_dc);
   if (err != DC1394_SUCCESS) msg_error(HERE, "BB2: reset failed");

   err = setStereoVideoCapture(&ws.pgrstereocamera);
   if (err != DC1394_SUCCESS) msg_error(HERE, "BB2: reset failed", false);

//    std::cout << "done - all good!" << std::endl;
//  }
}

void np::dc1394_capture(Bumblebee2WS& ws){

//   std::cout << "dc1394_capture" << std::endl;
  uint frame_length;
  dc1394video_frame_t* frame = NULL;
  dc1394error_t err = dc1394_capture_dequeue(ws.pgrstereocamera.camera, \
        DC1394_CAPTURE_POLICY_WAIT, &frame);
  if (err != DC1394_SUCCESS)
    msg_error(HERE, "cannot capture frame");

  frame_length = ws.pgrstereocamera.nRows*ws.pgrstereocamera.nCols*6;

  // check temporary buffer size
  if (ws.buffer_raw.N != frame_length)
    ws.buffer_raw.resize(ws.pgrstereocamera.nRows*2, \
                          ws.pgrstereocamera.nCols,3);

  // deinterlace it (separate left and right)
  dc1394_deinterlace_stereo(
                            frame->image,
                            ws.buffer_raw.p,
                            ws.pgrstereocamera.nCols,
                            ws.pgrstereocamera.nRows*2
                            );

  // check buffer size and resize, if necessary
  if (ws.buffer_rgb.N != frame_length)
    ws.buffer_rgb.resize(ws.pgrstereocamera.nRows*2, \
                            ws.pgrstereocamera.nCols,3);

  // extract color from the bayer tile image
  // note: this will alias colors on the top and bottom rows
  err = dc1394_bayer_decoding_8bit(
                                    ws.buffer_raw.p,
                                    ws.buffer_rgb.p,
                                    ws.pgrstereocamera.nCols,
                                    ws.pgrstereocamera.nRows*2,
                                    ws.pgrstereocamera.bayerTile,
                                    DC1394_BAYER_METHOD_BILINEAR
                                  );
  if (err != DC1394_SUCCESS)
    msg_error(HERE, "BB2: error during Bayer decoding");

  // return frame (memory) back to camera queue
  dc1394_capture_enqueue(ws.pgrstereocamera.camera, frame);
};

#else
// Dummy implementation
const char msg[] = "compile with -DNP_DC1394 to use a Bumblebee2";

namespace np {struct Bumblebee2WS {};};

np::Bumblebee2::Bumblebee2()
{msg_missing_implementation(HERE, msg);};

np::Bumblebee2::Bumblebee2()
{msg_missing_implementation(HERE, msg);};

np::Bumblebee2::~Bumblebee2()
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::init()
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::init(uint64_t identifier)
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::init(const char* identifier)
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::deinit()
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::start_capturing()
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::stop_capturing()
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::grab(byteA& bw, byteA& rgb)
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::grab(byteA& rgbL, byteA& rgbR)
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::load_parameters(const char *filename)
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::undistort(doubleA& xu, const doubleA& xd, CamId c)
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::distort(doubleA& xd, const doubleA& xu, CamId c)
{msg_missing_implementation(HERE, msg);};

void np::Bumblebee2::rectify(bool r)
{msg_missing_implementation(HERE, msg);};

bool np::Bumblebee2::is_stereo()
{msg_missing_implementation(HERE, msg); return false;};
#endif

#ifdef MT_BUMBLE
BumblebeeModule::BumblebeeModule():Process("BumblebeeProcess")
{};

void BumblebeeModule::open(){
  camera = new np::Bumblebee2();
  ifstream fil;
  MT::open(fil,"../../configurations/camera.cfg");
  calib.read(fil);
  step();
};

void BumblebeeModule::step(){
  byteA tmpL,tmpR;
  camera->grab(tmpL, tmpR);
  calib.rectifyImages(tmpL, tmpR);
  output.writeAccess(this);
  output.rgbL=tmpL;
  output.rgbR=tmpR;
  output.deAccess(this);
};

void BumblebeeModule::close(){
  camera->stop_capturing();
  camera->deinit();
  delete camera;
};
#endif
