#include <System/engine.h>
//#include <Gui/opengl.h>
#include <signal.h>
#include <thread>
#include <functional>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <omp.h>

#include <Core/util.h>
#include <Perception/audio.h>
#include <Perception/perception.h>
#include <Perception/videoEncoder.h>
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
}

class GrabAndSave {
private:
	const bool& terminated;
	bool ready;
	int id;
	MT::String name;
	FlycapInterface cam;
	VideoEncoder_x264_simple enc;
	TimeTagFile times;
	byteA buffer;
	double start_time;

public:
	GrabAndSave(int camID, const char* name, const MT::String& created, const bool& terminated) :
		terminated(terminated), ready(false), id(camID), name(name),
		cam(id, MLR::PIXEL_FORMAT_RAW8, MLR::PIXEL_FORMAT_RGB8),
		enc(STRING("z." << name << "." << created << ".264"), 60, 0, MLR::PIXEL_FORMAT_RGB8),
		times(enc.name()), start_time(ULONG_MAX) {
	}

	bool isReady() const {
		return ready;
	}
	void setActiveTime(double start_time) {
		this->start_time = start_time;
	}

	void step() {
		double timestamp;
		if((ready = cam.grab(buffer, timestamp, 500))) {
			if(timestamp >= start_time) {
				enc.addFrame(buffer);
				times.add_stamp(timestamp);
			}
		} else {
			cerr << "grab " << id << " failed" << endl;
		}
	}
};

class RecordingSystem {
private:
	MT::String created;
	GrabAndSave cam1, cam2;//, cam3;
	VideoEncoder_x264_simple kinect_video, kinect_depth;
	TimeTagFile kinect_video_times, kinect_depth_times;
	AudioWriter_libav audio_writer;
	AudioPoller_PA audio_poller;
	byteA audio_buf, kinect_depth_repack;
	KinectCallbackReceiver kinect;
	int kin_video_count, kin_depth_count;
	double start_time;

protected:

	void openmp_run() {
		kinect.startStreaming();
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
/*#pragma omp section
			while(!terminated)
				cam3.step();*/
#pragma omp section
			while(!terminated) {
				audio_poller.read(audio_buf);
				if(ready) {
					audio_writer.writeSamples_R48000_2C_S16_NE(audio_buf);
				}
			}
#pragma omp section
			while(!terminated && !ready) {
				if(cam1.isReady() && cam2.isReady() /*&& cam3.isReady()*/) {
					start_time = MT::clockTime();
					cam1.setActiveTime(start_time);
					cam2.setActiveTime(start_time);
					/*cam3.setActiveTime(start_time);*/
					ready = true;
				}
			}
		}
	}

	void kinect_video_cb(const byteA& rgb, double timestamp) {
		if(!terminated && (timestamp > start_time || (start_time - timestamp < .016))) {
			kinect_video.addFrame(rgb);
			kinect_video_times.add_stamp(timestamp);
		}
	}
	void kinect_depth_cb(const MT::Array<uint16_t>& depth, double timestamp) {
		if(!terminated && (timestamp > start_time || (start_time - timestamp < .016))) {
			MLR::pack_kindepth2rgb(depth, kinect_depth_repack);
			kinect_depth.addFrame(kinect_depth_repack);
			kinect_depth_times.add_stamp(timestamp);
		}
	}

public:
	RecordingSystem(int id1, int id2, int id3) :
		created(MT::getNowString()), cam1(id1, "cam1", created, terminated),
		cam2(id2, "cam2", created, terminated), /*cam3(id3, "cam3", created, terminated),*/
		kinect_video(STRING("z.kinect_rgb." << created <<".264"), 30, 0, MLR::PIXEL_FORMAT_RGB8),
		kinect_depth(STRING("z.kinect_depthRgb." << created <<".264"), 30, 0, MLR::PIXEL_FORMAT_RGB8),
		kinect_video_times(kinect_video.name()), kinect_depth_times(kinect_depth.name()),
		audio_writer(STRING("z.mike." << created << ".wav")),
		audio_buf(8192), kinect(std::bind(&RecordingSystem::kinect_depth_cb, this, _1, _2),
				std::bind(&RecordingSystem::kinect_video_cb, this, _1, _2)),
		kin_video_count(0), kin_depth_count(0), start_time(ULONG_MAX) {
	}

	void run() {
		std::thread runner(std::bind(&RecordingSystem::openmp_run, this));
		while(!terminated) {
			std::this_thread::sleep_for (std::chrono::milliseconds(100));
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
			MT::getParameter<int>("camID1"));
		s.run();
	} catch(const std::exception& ex) {
		cerr << ex.what() << endl;
	} catch(const char* msg) {
		cerr << msg << endl;
	}
    return 0;
}
