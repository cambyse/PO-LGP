//#include <System/engine.h>
//#include <Gui/opengl.h>
#include <signal.h>
#include <thread>
#include <functional>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <omp.h>

#include <Core/util.h>
#include <Core/thread.h>
#include <Perception/audio.h>
#include <Perception/perception.h>
#include <Perception/videoEncoder.h>
#include <Hardware/kinect/kinect.h>
#include <Hardware/ueyecamera/ueyecamera.h>
using namespace mlr;
using namespace std;
using namespace std::placeholders;

void lib_hardware_kinect();
void lib_hardware_ueyecamera();
void lib_Perception();

class TimeTagFile {
private:
	ofstream timeTagFile;
	unsigned int sequence_num;

public:
	TimeTagFile(const mlr::String& filename) : timeTagFile(STRING(filename << ".times")), sequence_num(0) {

	}
	void add_stamp(double timestamp) {
		timeTagFile << setw(6) << sequence_num++ << setprecision(14) << setw(20) << timestamp << endl;
	}
	void mark_missing() {
		sequence_num++;
	}
};

struct TomsyRecorderSystem{
    ACCESS(byteA, ueye_rgb_1)
    ACCESS(byteA, ueye_rgb_3)
    ACCESS(byteA, ueye_rgb_4)

    TomsyRecorderSystem(){
        addModule("KinectThread", NULL, /*Module::loopWithBeat,*/ .01); //this is callback driven...
        addModule<KinectDepthPacking>("KinectDepthPacking" /*,Module::listenFirst*/ );
        addModule("ImageViewer", "ImageViewer_rgb", {"kinect_rgb"} /*,Module::listenFirst*/ );
        addModule("ImageViewer", "ImageViewer_depth", {"kinect_depthRgb"} /*,Module::listenFirst*/ );
        //      addModule("Kinect2PointCloud", NULL, /*Module::loopWithBeat,*/ .2);
        //      addModule("PointCloudViewer", NULL, {"kinect_points", "kinect_pointColors"} /*,Module::listenFirst*/ );

        VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", {"kinect_rgb"} /*,Module::listenFirst*/ );
        m_enc->set_rgb(true);
        m_enc->set_fps(30);
        VideoEncoderX264 *m_denc = addModule<VideoEncoderX264>("VideoEncoder_depth", {"kinect_depthRgb"} /*,Module::listenFirst*/ );
        m_denc->set_fps(30);

        addModule("UEyePoller", "POLLER_1", {"ueye_rgb_1"}, Module::loopFull);
        VideoEncoderX264 *enc1 = addModule<VideoEncoderX264>("ENCODER_1", {"ueye_rgb_1"} /*,Module::listenFirst*/ );
        enc1->set_fps(60);
        addModule("ImageViewer", "VIEWER_1", {"ueye_rgb_1"} /*,Module::listenFirst*/ );

        addModule("UEyePoller", "POLLER_3", {"ueye_rgb_3"}, Module::loopFull);
        VideoEncoderX264 *enc3 = addModule<VideoEncoderX264>("ENCODER_3", {"ueye_rgb_3"} /*,Module::listenFirst*/ );
        enc3->set_fps(60);
        addModule("ImageViewer", "VIEWER_3", {"ueye_rgb_3"} /*,Module::listenFirst*/ );

        addModule("UEyePoller", "POLLER_4", {"ueye_rgb_4"}, Module::loopFull);
        VideoEncoderX264 *enc4 = addModule<VideoEncoderX264>("ENCODER_4", {"ueye_rgb_4"} /*,Module::listenFirst*/ );
        enc4->set_fps(60);
        addModule("ImageViewer", "VIEWER_4", {"ueye_rgb_4"} /*,Module::listenFirst*/ );

        addModule("AudioReader", "MIKE_1", Module::loopFull);
        addModule("AudioWriter", "WAV_1" /*,Module::listenFirst*/ );
        //connect();
    }
};

void threadedRun() {
    lib_hardware_kinect();
    lib_Perception();
    lib_hardware_ueyecamera();

    TomsyRecorderSystem S;

    //  cout <<S <<endl;

    //engine().enableAccessLog();
    threadOpenModules(true);

    moduleShutdown()->waitForStatusGreaterThan(0);

    threadCloseModules();
    cout <<"bye bye" <<endl;
}

bool terminated = false;
void got_signal(int) {
	terminated = true;
}

namespace {
class Condition {
public:

};

class GrabAndSave {
private:
	const bool& terminated;
	bool ready;
	int id;
	mlr::String name;
	UEyeInterface cam;
	VideoEncoder_x264_simple enc;
	TimeTagFile times;
	byteA buffer;
	double start_time;

public:
	GrabAndSave(int camID, const char* name, const mlr::String& created, const bool& terminated) :
		terminated(terminated), ready(false), id(camID), name(name), cam(id), enc(STRING("z." << name << "." << created << ".264")),
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

}

class RecordingSystem {
private:
	mlr::String created;
	GrabAndSave cam1, cam2, cam3;
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
					start_time = mlr::clockTime();
					cam1.setActiveTime(start_time);
					cam2.setActiveTime(start_time);
					cam3.setActiveTime(start_time);
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
	void kinect_depth_cb(const uint16A& depth, double timestamp) {
		if(!terminated && (timestamp > start_time || (start_time - timestamp < .016))) {
			mlr::pack_kindepth2rgb(depth, kinect_depth_repack);
			kinect_depth.addFrame(kinect_depth_repack);
			kinect_depth_times.add_stamp(timestamp);
		}
	}

public:
	RecordingSystem(int id1, int id2, int id3) :
		created(mlr::getNowString()), cam1(id1, "ueye1", created, terminated),
		cam2(id2, "ueye2", created, terminated), cam3(id3, "ueye3", created, terminated),
		kinect_video(STRING("z.kinect_rgb." << created <<".264")),
		kinect_depth(STRING("z.kinect_depthRgb." << created <<".264")),
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
		RecordingSystem s(mlr::getParameter<int>("POLLER_1_camID"),
			mlr::getParameter<int>("POLLER_3_camID"),
			mlr::getParameter<int>("POLLER_4_camID"));
		s.run();
	} catch(const UEyeException& ex) {
		cerr << ex.what() << endl;
	} catch(const std::exception& ex) {
		cerr << ex.what() << endl;
	} catch(const char* msg) {
		cerr << msg << endl;
	}
    return 0;
}
