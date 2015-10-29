#include <Core/array.h>
#include <Core/thread.h>
#include <Gui/opengl.h>
#include <iostream>
#include <functional>

#include "libfreenect.h"

using namespace std;
using namespace std::placeholders;

RWLock depthLock;
RWLock rgbLock;
uint16A depth;
byteA rgb, rgb_back;

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp){
        depthLock.writeLock();
  //	rgb.setCarray((byte*)v_rgb, rgbg.N);
        depth.setCarray((uint16_t*)v_depth, depth.N);
  //	pthread_cond_signal(&gl_frame_cond);
        depthLock.unlock();
}

void rgb_cb(freenect_device *dev, void *v_rgb, uint32_t timestamp){
	rgbLock.writeLock();
//	rgb.setCarray((byte*)v_rgb, rgbg.N);
	rgb = rgb_back;
//	pthread_cond_signal(&gl_frame_cond);
	rgbLock.unlock();
}

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;
volatile int die = 0;
pthread_t freenect_thread;

struct KinectThread:Thread{
  void open(){
  }
};

void *freenect_threadfunc(void *arg){
	freenect_set_tilt_degs(f_dev,freenect_angle);
	freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
	freenect_set_video_buffer(f_dev, rgb_back.p);

	freenect_start_depth(f_dev);
	freenect_start_video(f_dev);

	printf("'w' - tilt up, 's' - level, 'x' - tilt down, '0'-'6' - select LED mode, '+' & '-' - change IR intensity \n");
	printf("'f' - change video format, 'm' - mirror video, 'o' - rotate video with accelerometer \n");
	printf("'e' - auto exposure, 'b' - white balance, 'r' - raw color, 'n' - near mode (K4W only) \n");

	while (!die && freenect_process_events(f_ctx) >= 0) {
		//Throttle the text output
		if (accelCount++ >= 2000)
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

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);
//	freenect_set_led(f_dev,LED_GREEN);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	printf("-- done!\n");
	return NULL;
}


void direct(int user_device_number = 0){
  int res;

  OpenGL gl("FREENECT",640,480);

  depth.resize(480,640);
  rgb_back.resize(480,640,3);
  rgb.resize(480,640,3);

  printf("Kinect camera test\n");

  if (freenect_init(&f_ctx, NULL) < 0) {
          HALT("freenect_init() failed");
  }

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

  res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
  if (res) {
          freenect_shutdown(f_ctx);
          HALT("pthread_create failed\n");
  }

  for(uint t=0;t<200;t++){
    mlr::wait(.01);
    rgbLock.readLock();
    if(rgb.N){
      gl.background = rgb;
    }
    rgbLock.unlock();
    gl.update();
    cout <<rgb.dim() <<endl;
  }

  die = 1;
  pthread_join(freenect_thread, NULL);

}

int main(int argc, char* argv[]){
//  testStreamer();
  direct();
  return 0;
}
