#include <System/engine.h>
//#include <Gui/opengl.h>
#include <signal.h>
#include <thread>
#include <functional>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <omp.h>
#include <mutex>
#include <condition_variable>

#include <Core/util.h>
#include <Perception/audio.h>
#include <Perception/perception.h>
#include <Perception/videoEncoder.h>
#include <Perception/image_pub.h>
#include <Hardware/kinect/kinect.h>
#include <Hardware/flycap/flycap.h>

using namespace MLR;
using namespace std;
using namespace std::placeholders;

class TimeTagFile {
private:
	ofstream timeTagFile;
	unsigned int sequence_num;

public:
	TimeTagFile(const MT::String& filename) : timeTagFile(STRING(filename << ".times")), sequence_num(0) {

	}
	void add_stamp(double timestamp) {
		timeTagFile << setw(6) << sequence_num++ << setprecision(14) << setw(20) << timestamp << endl;
	}
	void mark_missing() {
		sequence_num++;
	}
};

bool terminated = false;
void got_signal(int) {
	terminated = true;
	ros_shutdown();
}

class VideoSave {
private:
	VideoEncoder_x264_simple enc;
	TimeTagFile times;
	double start_time;
	ImagePublisher pub;

protected:
	MT::String name;
	const bool& terminated;
	bool ready;
	//byteA buffer;
	OpenGL gl;

public:
	VideoSave(const char* name, const MT::String& created, const bool& terminated) :
		enc(STRING("z." << name << "." << created << ".264"), 60, 0, MLR::PIXEL_FORMAT_RGB8),
		times(enc.name()), start_time(ULONG_MAX), pub(name, name, MLR::PIXEL_FORMAT_RGB8),
		name(name), terminated(terminated), ready(false) {
	}

	bool isReady() const {
		return ready;
	}
	void setActiveTime(double start_time) {
		this->start_time = start_time;
	}

	void add_frame(const byteA& frame, double timestamp, bool copy_gl = false) {
		if(timestamp >= start_time) {
			enc.addFrame(frame);
			times.add_stamp(timestamp);
		}
		double w_ratio = (double)gl.width / (double)gl.background.d0;
		double h_ratio = (double)gl.height / (double)gl.background.d1;
		gl.backgroundZoom = min(w_ratio, h_ratio);
		if(copy_gl)
			gl.background = frame;
		gl.update(NULL, false, false, false);
		pub.publish(frame, timestamp);
	}

};

class FlycapGrabAndSave : public VideoSave {
private:
	FlycapInterface cam;

public:
	FlycapGrabAndSave(int id, const char* name, const MT::String& created, const bool& terminated) :
		VideoSave(name, created, terminated),
		cam(id, MLR::PIXEL_FORMAT_RAW8, MLR::PIXEL_FORMAT_RGB8) {
	}
	virtual ~FlycapGrabAndSave() {

	}

	void step() {
		double timestamp;
		if((ready = cam.grab(gl.background, timestamp, 500))) {
			add_frame(gl.background, timestamp);
		} else {
			cerr << "grab " << name << " failed" << endl;
		}
	}
};
class KinectGrabAndSave {
private:
	KinectCallbackReceiver kinect;
	VideoSave vs_rgb, vs_depth;
	byteA depth_image;

protected:
	void kinect_video_cb(const byteA& rgb, double timestamp) {
		vs_rgb.add_frame(rgb, timestamp, true);
	}

	void kinect_depth_cb(const MT::Array<uint16_t>& depth, double timestamp) {
		MLR::pack_kindepth2rgb(depth, depth_image);
		vs_depth.add_frame(depth_image, timestamp, true);
	}

public:
	KinectGrabAndSave(int id, const char* name, const MT::String& created, const bool& terminated) :
		kinect(std::bind(&KinectGrabAndSave::kinect_depth_cb, this, _1, _2),
			   std::bind(&KinectGrabAndSave::kinect_video_cb, this, _1, _2), id),
	    vs_rgb(STRING(name << "_rgb").p, created, terminated),
	    vs_depth(STRING(name << "_depth").p, created, terminated) {
	}

	void setActiveTime(double start_time) {
		vs_rgb.setActiveTime(start_time - .016);
		vs_depth.setActiveTime(start_time - .016);
	}
	void startStreaming() {
		kinect.startStreaming();
	}
	void stopStreaming() {
		kinect.stopStreaming();
	}
};

class RecordingSystem {
private:
	MT::String created;
	FlycapGrabAndSave cam1, cam2, cam3;
	KinectGrabAndSave front_kinect;
	AudioWriter_libav audio_writer;
	AudioPoller_PA audio_poller;
	byteA audio_buf;
	double start_time;

protected:

	void openmp_run() {
		front_kinect.startStreaming();
		//side_kinect.startStreaming();
		bool ready = false;

		// make sure everything is running smoothly before starting to record
#pragma omp parallel sections num_threads(5)
		{
#pragma omp section
			while(!terminated)
				cam1.step();
#pragma omp section
			while(!terminated)
				cam2.step();
#pragma omp section
			while(!terminated)
				cam3.step();
#pragma omp section
			while(!terminated) {
				audio_poller.read(audio_buf);
				if(ready) {
					audio_writer.writeSamples_R48000_2C_S16_NE(audio_buf);
				}
			}
#pragma omp section
			while(!terminated && !ready) {
				if(cam1.isReady() && cam2.isReady() && cam3.isReady()) {
					start_time = MT::clockTime();
					cam1.setActiveTime(start_time);
					cam2.setActiveTime(start_time);
					cam3.setActiveTime(start_time);
					front_kinect.setActiveTime(start_time);
					ready = true;
				}
			}
		}
	}


public:
	RecordingSystem(int id1, int id2, int id3, int kinID1, int kinID2) :
		created(MT::getNowString()),
		cam1(id1, STRING("pg_cam_" << id1).p, created, terminated),
		cam2(id2, STRING("pg_cam_" << id2).p, created, terminated),
		cam3(id3, STRING("pg_cam_" << id3).p, created, terminated),
		front_kinect(kinID1, "front_kinect", created, terminated),
		audio_writer(STRING("z.mike." << created << ".wav")),
		audio_buf(8192),
		start_time(ULONG_MAX) {
	}

	void run() {
		std::thread runner(std::bind(&RecordingSystem::openmp_run, this));
		while(!terminated && process_image_callbacks()) {
			std::this_thread::sleep_for (std::chrono::milliseconds(50));
		}
		runner.join();
	}
};

void test_openmp() {
#pragma omp parallel num_threads(5)
#pragma omp sections
	{
#pragma omp section
	while(!terminated) {
		std::this_thread::sleep_for (std::chrono::seconds(1));
		std::clog << "1" << endl;
	}
#pragma omp section
	while(!terminated) {
		std::this_thread::sleep_for (std::chrono::milliseconds(500));
		std::clog << "2" << endl;
	}
	}
}

int main(int argc,char **argv){
	init_image_publishers(argc, argv, "third_hand_recorder");

	struct sigaction sa;
	memset( &sa, 0, sizeof(sa) );
	sa.sa_handler = got_signal;
	sigfillset(&sa.sa_mask);
	sigaction(SIGINT,&sa,NULL);
    //threadedRun();

	omp_set_nested(true);

	//test_openmp();



	try {
		RecordingSystem s(MT::getParameter<int>("camID1"),
			MT::getParameter<int>("camID2"),
			MT::getParameter<int>("camID3"),
			MT::getParameter<int>("kinectID1"),
			MT::getParameter<int>("kinectID2"));
		s.run();
	} catch(const std::exception& ex) {
		cerr << ex.what() << endl;
	} catch(const char* msg) {
		cerr << msg << endl;
	}
    return 0;
}
