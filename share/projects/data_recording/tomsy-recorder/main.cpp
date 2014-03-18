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
#include <Core/module.h>
#include <Perception/audio.h>
#include <Perception/perception.h>
#include <Perception/videoEncoder.h>
#include <Hardware/kinect/kinect.h>
#include <Hardware/ueyecamera/ueyecamera.h>
using namespace MLR;
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
	TimeTagFile(const MT::String& filename) : timeTagFile(STRING(filename << ".times")), sequence_num(0) {

	}
	void add_stamp(double timestamp) {
		timeTagFile << setw(6) << sequence_num++ << setprecision(14) << setw(20) << timestamp << endl;
	}
	void mark_missing() {
		sequence_num++;
	}
};

struct TomsyRecorderSystem:System{
    ACCESS(byteA, ueye_rgb_1)
    ACCESS(byteA, ueye_rgb_3)
    ACCESS(byteA, ueye_rgb_4)

    TomsyRecorderSystem(){
        addModule("KinectPoller", NULL, Module_Thread::loopWithBeat, .01); //this is callback driven...
        addModule<KinectDepthPacking>("KinectDepthPacking", Module_Thread::listenFirst);
        addModule("ImageViewer", "ImageViewer_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
        addModule("ImageViewer", "ImageViewer_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
        //      addModule("Kinect2PointCloud", NULL, Module_Thread::loopWithBeat, .2);
        //      addModule("PointCloudViewer", NULL, STRINGS("kinect_points", "kinect_pointColors"), Module_Thread::listenFirst);

        VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", STRINGS("kinect_rgb"), Module_Thread::listenFirst);
        m_enc->set_rgb(true);
        m_enc->set_fps(30);
        VideoEncoderX264 *m_denc = addModule<VideoEncoderX264>("VideoEncoder_depth", STRINGS("kinect_depthRgb"), Module_Thread::listenFirst);
        m_denc->set_fps(30);

        addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), Module_Thread::loopFull);
        VideoEncoderX264 *enc1 = addModule<VideoEncoderX264>("ENCODER_1", STRINGS("ueye_rgb_1"), Module_Thread::listenFirst);
        enc1->set_fps(60);
        addModule("ImageViewer", "VIEWER_1", STRINGS("ueye_rgb_1"), Module_Thread::listenFirst);

        addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), Module_Thread::loopFull);
        VideoEncoderX264 *enc3 = addModule<VideoEncoderX264>("ENCODER_3", STRINGS("ueye_rgb_3"), Module_Thread::listenFirst);
        enc3->set_fps(60);
        addModule("ImageViewer", "VIEWER_3", STRINGS("ueye_rgb_3"), Module_Thread::listenFirst);

        addModule("UEyePoller", "POLLER_4", STRINGS("ueye_rgb_4"), Module_Thread::loopFull);
        VideoEncoderX264 *enc4 = addModule<VideoEncoderX264>("ENCODER_4", STRINGS("ueye_rgb_4"), Module_Thread::listenFirst);
        enc4->set_fps(60);
        addModule("ImageViewer", "VIEWER_4", STRINGS("ueye_rgb_4"), Module_Thread::listenFirst);

        addModule("AudioReader", "MIKE_1", Module_Thread::loopFull);
        addModule("AudioWriter", "WAV_1", Module_Thread::listenFirst);
        connect();
    }
};

void threadedRun() {
    lib_hardware_kinect();
    lib_Perception();
    lib_hardware_ueyecamera();

    TomsyRecorderSystem S;

    //  cout <<S <<endl;

    engine().enableAccessLog();
    engine().open(S);

    engine().shutdown.waitForSignal();

    engine().close(S);
    cout <<"bye bye" <<endl;
}

bool terminated = false;
void got_signal(int) {
	terminated = true;
}

namespace {
void grab_one(const char* id, UEyeInterface& cam, VideoEncoder_x264_simple& enc, byteA& buf, TimeTagFile& times) {
	double timestamp;
	if(cam.grab(buf, timestamp, 500)) {
		enc.addFrame(buf);
		times.add_stamp(timestamp);
	} else {
		times.mark_missing();
		cerr << "grab " << id << " failed" << endl;
	}
}

class GrabAndSave {
private:
	const bool& terminated;
	int id;
	UEyeInterface cam;
	VideoEncoder_x264_simple enc;
	TimeTagFile times;
	byteA buffer;
	Mutex start_lock;

public:
	GrabAndSave(int camID, const char* name, const MT::String& created, const bool& terminated) :
		terminated(terminated), id(camID), cam(id), enc(STRING("z." << name << created << ".264")),
		times(enc.name()) {
	}

	bool preroll() {
		// parallel start of streaming reliably crashes the ueye daemon...
		{
			Lock l(start_lock);
			cam.startStreaming();
		}
		double timestamp;
		while(!terminated) {
			if(cam.grab(buffer, timestamp))
				return true;
		}
		return false;
	}

	void run(const double& start_time) {
		double timestamp;
		while(!terminated) {
			if(cam.grab(buffer, timestamp, 500)) {
				if(timestamp > start_time) {
					enc.addFrame(buffer);
					times.add_stamp(timestamp);
				}
			} else {
				cerr << "grab " << id << " failed" << endl;
			}
		}
	}
};

}

class RecordingSystem {
private:
	MT::String created;
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

		bool cam1_ready = false, cam2_ready = false, cam3_ready = false;

#pragma omp parallel num_threads(4)
		// make sure everything is running smoothly before starting to record
#pragma omp sections
		{
#pragma omp section
		cam1_ready = cam1.preroll();
#pragma omp section
		cam2_ready = cam2.preroll();
#pragma omp section
		cam3_ready = cam3.preroll();
		}

		if(!cam1_ready || !cam2_ready || !cam3_ready) {
			cerr << "Cameras not initialized correctly, cam1_ready=" << cam1_ready
					<< ", cam2_ready=" << cam2_ready << ", cam3_ready=" << cam3_ready << endl;
			return;
		}
		start_time = MT::clockTime();

#pragma omp sections
		{
#pragma omp section
			cam1.run(start_time);
#pragma omp section
			cam2.run(start_time);
#pragma omp section
			cam3.run(start_time);
#pragma omp section
			{
				while(!terminated) {
					audio_poller.read(audio_buf);
					audio_writer.writeSamples_R48000_2C_S16_NE(audio_buf);
				}
			}
		}
	}

	void kinect_video_cb(const byteA& rgb, double timestamp) {
		if(timestamp > start_time) {
			kinect_video.addFrame(rgb);
			kinect_video_times.add_stamp(timestamp);
		}
	}
	void kinect_depth_cb(const MT::Array<uint16_t>& depth, double timestamp) {
		if(timestamp > start_time) {
			MLR::pack_kindepth2rgb(depth, kinect_depth_repack);
			kinect_depth.addFrame(kinect_depth_repack);
			kinect_depth_times.add_stamp(timestamp);
		}
	}

public:
	RecordingSystem(int id1, int id2, int id3) :
		created(MT::getNowString()), cam1(id1, "ueye1", created, terminated),
		cam2(id2, "ueye2", created, terminated), cam3(id3, "ueye3", created, terminated),
		kinect_video(STRING("z.kinect_rgb." << created <<".264")),
		kinect_depth(STRING("z.kinect_depthRgb." << created <<".264")),
		kinect_video_times(kinect_video.name()), kinect_depth_times(kinect_depth.name()),
		audio_writer(STRING("z.mike." << created << ".wav")),
		audio_buf(8192), kinect(std::bind(&RecordingSystem::kinect_depth_cb, this, _1, _2),
				std::bind(&RecordingSystem::kinect_video_cb, this, _1, _2)),
		kin_video_count(0), kin_depth_count(0) {
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
		RecordingSystem s(MT::getParameter<int>("POLLER_1_camID"),
			MT::getParameter<int>("POLLER_3_camID"),
			MT::getParameter<int>("POLLER_4_camID"));
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
