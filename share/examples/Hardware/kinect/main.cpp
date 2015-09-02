#include <Core/array.h>
#include <Core/thread.h>
#include <Core/module.h>
#include <Gui/opengl.h>

#include "libfreenect.h"

//int freenect_led;

struct KinectThread:Module{
  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
  freenect_context *f_ctx;
  freenect_device *f_dev;
  int user_device_number = 0;
  uint16A depth_buffer;
  byteA rgb_buffer;
  uint accelCount=0;
  freenect_video_format requested_format = FREENECT_VIDEO_RGB;
  freenect_video_format current_format = FREENECT_VIDEO_RGB;

  KinectThread():Module("KinThread"){
    kinect_rgb.createVariable();
    kinect_depth.createVariable();
    depth_buffer.resize(480,640);
    rgb_buffer.resize(480,640,3);
    single=this;
  }

  static KinectThread* single;
  static void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp){
    single->kinect_depth.set() = single->depth_buffer;
  }

  static void rgb_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp){
    single->kinect_rgb.set() = single->rgb_buffer;
  }

  void dummy(char key){
    int tilt_changed;
    int freenect_angle = 0;

    if (key == 'w') {
      freenect_angle++;
      if (freenect_angle > 30) {
        freenect_angle = 30;
      }
      tilt_changed++;
    }
    if (key == 's') {
      freenect_angle = 0;
      tilt_changed++;
    }
    if (key == 'f') {
      if (requested_format == FREENECT_VIDEO_IR_8BIT)
        requested_format = FREENECT_VIDEO_RGB;
      else if (requested_format == FREENECT_VIDEO_RGB)
        requested_format = FREENECT_VIDEO_YUV_RGB;
      else
        requested_format = FREENECT_VIDEO_IR_8BIT;
    }
    if (key == 'x') {
      freenect_angle--;
      if (freenect_angle < -30) {
        freenect_angle = -30;
      }
      tilt_changed++;
    }
    if (key == 'e') {
      static freenect_flag_value auto_exposure = FREENECT_ON;
      freenect_set_flag(f_dev, FREENECT_AUTO_EXPOSURE, auto_exposure);
      auto_exposure = auto_exposure ? FREENECT_OFF : FREENECT_ON;
    }
    if (key == 'b') {
      static freenect_flag_value white_balance = FREENECT_ON;
      freenect_set_flag(f_dev, FREENECT_AUTO_WHITE_BALANCE, white_balance);
      white_balance = white_balance ? FREENECT_OFF : FREENECT_ON;
    }
    if (key == 'r') {
      static freenect_flag_value raw_color = FREENECT_ON;
      freenect_set_flag(f_dev, FREENECT_RAW_COLOR, raw_color);
      raw_color = raw_color ? FREENECT_OFF : FREENECT_ON;
    }
    if (key == 'm') {
      static freenect_flag_value mirror = FREENECT_ON;
      freenect_set_flag(f_dev, FREENECT_MIRROR_DEPTH, mirror);
      freenect_set_flag(f_dev, FREENECT_MIRROR_VIDEO, mirror);
      mirror = mirror ? FREENECT_OFF : FREENECT_ON;
    }
    if (key == 'n') {
      static freenect_flag_value near_mode = FREENECT_ON;
      freenect_set_flag(f_dev, FREENECT_NEAR_MODE, near_mode);
      near_mode = near_mode ? FREENECT_OFF : FREENECT_ON;
    }

    if (key == '+') {
      uint16_t brightness = freenect_get_ir_brightness(f_dev) + 2;
      freenect_set_ir_brightness(f_dev, brightness);
    }
    if (key == '-') {
      uint16_t brightness = freenect_get_ir_brightness(f_dev) - 2;
      freenect_set_ir_brightness(f_dev, brightness);
    }

    if (key == '1') {
      freenect_set_led(f_dev,LED_GREEN);
    }
    if (key == '2') {
      freenect_set_led(f_dev,LED_RED);
    }
    if (key == '3') {
      freenect_set_led(f_dev,LED_YELLOW);
    }
    if (key == '4') {
      freenect_set_led(f_dev,LED_BLINK_GREEN);
    }
    if (key == '5') {
      // 5 is the same as 4
      freenect_set_led(f_dev,LED_BLINK_GREEN);
    }
    if (key == '6') {
      freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);
    }
    if (key == '0') {
      freenect_set_led(f_dev,LED_OFF);
    }

    if (tilt_changed) {
      freenect_set_tilt_degs(f_dev, freenect_angle);
      tilt_changed = 0;
    }
  }


  void open(){

    if(freenect_init(&f_ctx, NULL) < 0) HALT("freenect_init() failed");

    freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    int nr_devices = freenect_num_devices (f_ctx);
    printf ("Number of devices found: %d\n", nr_devices);


    if (nr_devices < 1) {
      freenect_shutdown(f_ctx);
      HALT("Not enough devices");
    }

    if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
      freenect_shutdown(f_ctx);
      HALT("Could not open device\n");
    }


    freenect_set_tilt_degs(f_dev, 0);
    freenect_set_led(f_dev,LED_RED);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    freenect_set_video_buffer(f_dev, rgb_buffer.p);
    freenect_set_depth_buffer(f_dev, depth_buffer.p);

    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);
  }
  void step(){
    if(freenect_process_events(f_ctx) >= 0) {
      //Throttle the text output
                  if (accelCount++ >= 100)
                  {
                          accelCount = 0;
                          freenect_raw_tilt_state* state;
                          freenect_update_tilt_state(f_dev);
                          state = freenect_get_tilt_state(f_dev);
                          double dx,dy,dz;
                          freenect_get_mks_accel(state, &dx, &dy, &dz);
                          printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, dx, dy, dz);
                          fflush(stdout);
                  }

      if (requested_format != current_format) {
        freenect_stop_video(f_dev);
        freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
        freenect_start_video(f_dev);
        current_format = requested_format;
      }
    }
  }

  void close(){
    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);
    freenect_set_led(f_dev,LED_GREEN);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
  }
};

KinectThread *KinectThread::single= NULL;

void testKinect(){

  OpenGL gl("FREENECT",640,480);

  KinectThread kin;
  kin.threadLoop();

  for(uint t=0;t<200;t++){
    kin.kinect_rgb.waitForNextRevision();
    gl.background = kin.kinect_rgb.get();
    gl.update();
    cout <<'.' <<std::flush;
  }

  kin.threadClose();
}

int main(int argc, char* argv[]){
  testKinect();
  return 0;
}
