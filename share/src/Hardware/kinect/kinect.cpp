#include "kinect.h"

#include <libfreenect.h>


struct sKinectThread{
  KinectThread* kin;
  freenect_context *f_ctx;
  freenect_device *f_dev;
  int user_device_number = 0;
  uint16A depth_buffer;
  byteA rgb_buffer;
  uint accelCount=0;
  int freenect_angle = 0;
  freenect_video_format requested_format = FREENECT_VIDEO_RGB;
  freenect_video_format current_format = FREENECT_VIDEO_RGB;
  static void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp);

  static void rgb_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp);
};

sKinectThread* single=NULL;

KinectThread::KinectThread(ModuleL& system):Module("KinectThread", system, loopFull), verbose(0){
  s = new sKinectThread;
  s->kin = this;
  s->depth_buffer.resize(480,640);
  s->rgb_buffer.resize(480,640,3);
  if(single) HALT("just one for now!!");
  single=s;
}

KinectThread::~KinectThread(){
  delete s;
  single=NULL;
}


void sKinectThread::depth_cb(freenect_device* dev, void* v_depth, uint32_t timestamp){
  single->kin->kinect_depth.set(mlr::clockTime()) = single->depth_buffer;
}


void sKinectThread::rgb_cb(freenect_device* dev, void* v_rgb, uint32_t timestamp){
  single->kin->kinect_rgb.set(mlr::clockTime()) = single->rgb_buffer;
}


void KinectThread::glViewKeys(char key){
  int tilt_changed = 0;

  if (key == 'w') {
    s->freenect_angle++;
    if (s->freenect_angle > 30) {
      s->freenect_angle = 30;
    }
    tilt_changed++;
  }
  if (key == 's') {
    s->freenect_angle = 0;
    tilt_changed++;
  }
  if (key == 'f') {
    if (s->requested_format == FREENECT_VIDEO_IR_8BIT)
      s->requested_format = FREENECT_VIDEO_RGB;
    else if (s->requested_format == FREENECT_VIDEO_RGB)
      s->requested_format = FREENECT_VIDEO_YUV_RGB;
    else
      s->requested_format = FREENECT_VIDEO_IR_8BIT;
  }
  if (key == 'x') {
    s->freenect_angle--;
    if (s->freenect_angle < -30) {
      s->freenect_angle = -30;
    }
    tilt_changed++;
  }
  if (key == 'e') {
    static freenect_flag_value auto_exposure = FREENECT_ON;
    freenect_set_flag(s->f_dev, FREENECT_AUTO_EXPOSURE, auto_exposure);
    auto_exposure = auto_exposure ? FREENECT_OFF : FREENECT_ON;
  }
  if (key == 'b') {
    static freenect_flag_value white_balance = FREENECT_ON;
    freenect_set_flag(s->f_dev, FREENECT_AUTO_WHITE_BALANCE, white_balance);
    white_balance = white_balance ? FREENECT_OFF : FREENECT_ON;
  }
  if (key == 'r') {
    static freenect_flag_value raw_color = FREENECT_ON;
    freenect_set_flag(s->f_dev, FREENECT_RAW_COLOR, raw_color);
    raw_color = raw_color ? FREENECT_OFF : FREENECT_ON;
  }
  if (key == 'm') {
    static freenect_flag_value mirror = FREENECT_ON;
    freenect_set_flag(s->f_dev, FREENECT_MIRROR_DEPTH, mirror);
    freenect_set_flag(s->f_dev, FREENECT_MIRROR_VIDEO, mirror);
    mirror = mirror ? FREENECT_OFF : FREENECT_ON;
  }
  // if (key == 'n') {
  //   static freenect_flag_value near_mode = FREENECT_ON;
  //   freenect_set_flag(s->f_dev, FREENECT_NEAR_MODE, near_mode);
  //   near_mode = near_mode ? FREENECT_OFF : FREENECT_ON;
  // }

  // if (key == '+') {
  //   uint16_t brightness = freenect_get_ir_brightness(s->f_dev) + 2;
  //   freenect_set_ir_brightness(s->f_dev, brightness);
  // }
  // if (key == '-') {
  //   uint16_t brightness = freenect_get_ir_brightness(s->f_dev) - 2;
  //   freenect_set_ir_brightness(s->f_dev, brightness);
  // }

  if (key == '1') {
    freenect_set_led(s->f_dev,LED_GREEN);
  }
  if (key == '2') {
    freenect_set_led(s->f_dev,LED_RED);
  }
  if (key == '3') {
    freenect_set_led(s->f_dev,LED_YELLOW);
  }
  if (key == '4') {
    freenect_set_led(s->f_dev,LED_BLINK_GREEN);
  }
  if (key == '5') {
    // 5 is the same as 4
    freenect_set_led(s->f_dev,LED_BLINK_GREEN);
  }
  if (key == '6') {
    freenect_set_led(s->f_dev,LED_BLINK_RED_YELLOW);
  }
  if (key == '0') {
    freenect_set_led(s->f_dev,LED_OFF);
  }

  if (tilt_changed) {
    freenect_set_tilt_degs(s->f_dev, s->freenect_angle);
    tilt_changed = 0;
  }
}


void KinectThread::open(){

  if(freenect_init(&s->f_ctx, NULL) < 0) HALT("freenect_init() failed");

  if(verbose){
    freenect_set_log_level(s->f_ctx, FREENECT_LOG_DEBUG);
  }else{
    freenect_set_log_level(s->f_ctx, FREENECT_LOG_NOTICE);
  }
  freenect_select_subdevices(s->f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

  int nr_devices = freenect_num_devices (s->f_ctx);
  printf ("Number of devices found: %d\n", nr_devices);


  if (nr_devices < 1) {
    freenect_shutdown(s->f_ctx);
    HALT("Not enough devices");
  }

  if (freenect_open_device(s->f_ctx, &s->f_dev, s->user_device_number) < 0) {
    freenect_shutdown(s->f_ctx);
    HALT("Could not open device\n");
  }


  freenect_set_tilt_degs(s->f_dev, 0);
  freenect_set_led(s->f_dev,LED_RED);
  freenect_set_depth_callback(s->f_dev, s->depth_cb);
  freenect_set_video_callback(s->f_dev, s->rgb_cb);
  freenect_set_video_mode(s->f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, s->current_format));
  freenect_set_depth_mode(s->f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
  freenect_set_video_buffer(s->f_dev, s->rgb_buffer.p);
  freenect_set_depth_buffer(s->f_dev, s->depth_buffer.p);

  freenect_start_depth(s->f_dev);
  freenect_start_video(s->f_dev);
}


void KinectThread::step(){
  if(freenect_process_events(s->f_ctx) >= 0) {
    //Throttle the text output
    if (verbose && s->accelCount++ >= 100)
    {
      s->accelCount = 0;
      freenect_raw_tilt_state* state;
      freenect_update_tilt_state(s->f_dev);
      state = freenect_get_tilt_state(s->f_dev);
      double dx,dy,dz;
      freenect_get_mks_accel(state, &dx, &dy, &dz);
      printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
      fflush(stdout);
    }

    if (s->requested_format != s->current_format) {
      freenect_stop_video(s->f_dev);
      freenect_set_video_mode(s->f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, s->requested_format));
      freenect_start_video(s->f_dev);
      s->current_format = s->requested_format;
    }
  }
}


void KinectThread::close(){
  freenect_stop_depth(s->f_dev);
  freenect_stop_video(s->f_dev);
  freenect_set_led(s->f_dev,LED_GREEN);

  freenect_close_device(s->f_dev);
  freenect_shutdown(s->f_ctx);
}
