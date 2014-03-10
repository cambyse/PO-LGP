#include <System/engine.h>
//#include <Gui/opengl.h>
#include <signal.h>
#include <thread>
#include <functional>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <omp.h>

#include <Perception/audio.h>
#include <Perception/perception.h>
#include <Perception/videoEncoder.h>
#include <Hardware/kinect/kinect.h>
#include <Hardware/ueyecamera/ueyecamera.h>
using namespace MLR;
using namespace std;

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
		timeTagFile << setw(6) << sequence_num++ << setprecision(20) << setw(20) << timestamp << endl;
	}
};

struct TomsyRecorderSystem:System{
    ACCESS(byteA, ueye_rgb_1)
    ACCESS(byteA, ueye_rgb_3)
    ACCESS(byteA, ueye_rgb_4)

    TomsyRecorderSystem(){
        addModule("KinectPoller", NULL, ModuleThread::loopWithBeat, .01); //this is callback driven...
        addModule<KinectDepthPacking>("KinectDepthPacking", ModuleThread::listenFirst);
        addModule("ImageViewer", "ImageViewer_rgb", STRINGS("kinect_rgb"), ModuleThread::listenFirst);
        addModule("ImageViewer", "ImageViewer_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);
        //      addModule("Kinect2PointCloud", NULL, ModuleThread::loopWithBeat, .2);
        //      addModule("PointCloudViewer", NULL, STRINGS("kinect_points", "kinect_pointColors"), ModuleThread::listenFirst);

        VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", STRINGS("kinect_rgb"), ModuleThread::listenFirst);
        m_enc->set_rgb(true);
        m_enc->set_fps(30);
        VideoEncoderX264 *m_denc = addModule<VideoEncoderX264>("VideoEncoder_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);
        m_denc->set_fps(30);

        addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), ModuleThread::loopFull);
        VideoEncoderX264 *enc1 = addModule<VideoEncoderX264>("ENCODER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);
        enc1->set_fps(60);
        addModule("ImageViewer", "VIEWER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), ModuleThread::loopFull);
        VideoEncoderX264 *enc3 = addModule<VideoEncoderX264>("ENCODER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);
        enc3->set_fps(60);
        addModule("ImageViewer", "VIEWER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_4", STRINGS("ueye_rgb_4"), ModuleThread::loopFull);
        VideoEncoderX264 *enc4 = addModule<VideoEncoderX264>("ENCODER_4", STRINGS("ueye_rgb_4"), ModuleThread::listenFirst);
        enc4->set_fps(60);
        addModule("ImageViewer", "VIEWER_4", STRINGS("ueye_rgb_4"), ModuleThread::listenFirst);

        addModule("AudioReader", "MIKE_1", ModuleThread::loopFull);
        addModule("AudioWriter", "WAV_1", ModuleThread::listenFirst);
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
		cerr << "grab " << id << " failed" << endl;
	}
}
}

class RecordingSystem {
private:
	UEyeInterface cam1, cam2, cam3;
	VideoEncoder_x264_simple enc1, enc2, enc3;
	TimeTagFile times1, times2, times3;
	byteA buf1, buf2, buf3, audio_buf;
	AudioWriter_libav audio_writer;
	AudioPoller_PA audio_poller;

protected:

	void openmp_run() {
		timespec start, stop;
		// doing the start in parallel seems to cause lock-ups, so lets do it serially
		// TODO: time-sync grab start more accurately
		cam1.startStreaming();
		cam2.startStreaming();
		cam3.startStreaming();

#pragma omp parallel num_threads(4)
#pragma omp sections
		{
#pragma omp section
			{
				while(!terminated)
					grab_one("1", cam1, enc1, buf1, times1);
			}
#pragma omp section
			{
				while(!terminated)
					grab_one("2", cam2, enc2, buf2, times2);
			}
#pragma omp section
			{
				while(!terminated)
					grab_one("3", cam3, enc3, buf3, times3);
			}
#pragma omp section
			{
				while(!terminated) {
					audio_poller.read(audio_buf);
					audio_writer.writeSamples_R48000_2C_S16_NE(audio_buf);
				}
			}
		}
	}


public:
	RecordingSystem(int id1, int id2, int id3) : cam1(id1), cam2(id2), cam3(id3),
		enc1(STRING("z.ueye1." <<MT::getNowString() <<".264"), 60),
		enc2(STRING("z.ueye2." <<MT::getNowString() <<".264"), 60),
		enc3(STRING("z.ueye3." <<MT::getNowString() <<".264"), 60),
		times1(enc1.name()), times2(enc2.name()), times3(enc3.name()),
		audio_writer(STRING("z.mike." << MT::getNowString() << ".wav")),
		audio_buf(8192) {
	}

	void run() {
		std::thread runner(std::bind(&RecordingSystem::openmp_run, this));
		while(!terminated) {
			std::this_thread::sleep_for (std::chrono::seconds(1));
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
	}
    return 0;
}
