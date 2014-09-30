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
		name(name), terminated(terminated), ready(false), gl(name) {
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
//		gl.update(NULL, false, false, false);
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

namespace {
	void run(FlycapGrabAndSave* cam) {
		while(!terminated)
			cam->step();
	}
}

int main(int argc,char **argv){
	omp_set_nested(true);

	init_image_publishers(argc, argv, "third_hand_recorder");

	int id1 = MT::getParameter<int>("camID1"),
			id2 =  MT::getParameter<int>("camID2"),
			id3 = MT::getParameter<int>("camID3"),
			id4 = MT::getParameter<int>("camID4");
	MT::String created(MT::getNowString());
	FlycapGrabAndSave cam1(id1, STRING("pg_cam_" << id1).p, created, terminated);
	cam1.setActiveTime(MT::clockTime());
	FlycapGrabAndSave cam2(id2, STRING("pg_cam_" << id2).p, created, terminated);
	cam2.setActiveTime(MT::clockTime());
	FlycapGrabAndSave cam3(id3, STRING("pg_cam_" << id3).p, created, terminated);
	cam3.setActiveTime(MT::clockTime());
	FlycapGrabAndSave cam4(id4, STRING("pg_cam_" << id4).p, created, terminated);
	cam4.setActiveTime(MT::clockTime());

	std::thread runner1(std::bind(&run, &cam1));
	std::thread runner2(std::bind(&run, &cam2));
	std::thread runner3(std::bind(&run, &cam3));
	std::thread runner4(std::bind(&run, &cam4));
	runner1.join();

	return 0;
}
