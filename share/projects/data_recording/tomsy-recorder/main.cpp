#include <System/engine.h>
//#include <Gui/opengl.h>
//#include <signal.h>
//#include <sys/time.h>

#include <Perception/perception.h>
#include <Hardware/kinect/kinect.h>

void lib_hardware_kinect();
void lib_hardware_ueyecamera();
void lib_Perception();

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
        VideoEncoderX264 *m_denc = addModule<VideoEncoderX264>("VideoEncoder_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);
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

int main(int argc,char **argv){
    threadedRun();
    return 0;
}
