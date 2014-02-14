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
        addModule("KinectPoller", NULL, ModuleThread::loopWithBeat, .01); //this is callback driven...
        addModule<KinectDepthPacking>("KinectDepthPacking", ModuleThread::listenFirst);
        addModule("ImageViewer", "ImageViewer_rgb", STRINGS("kinect_rgb"), ModuleThread::listenFirst);
        addModule("ImageViewer", "ImageViewer_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);
        //      addModule("Kinect2PointCloud", NULL, ModuleThread::loopWithBeat, .2);
        //      addModule("PointCloudViewer", NULL, STRINGS("kinect_points", "kinect_pointColors"), ModuleThread::listenFirst);

        VideoEncoder *m_enc = addModule<VideoEncoder>("VideoEncoder_rgb", STRINGS("kinect_rgb"), ModuleThread::listenFirst);
        m_enc->set_rgb(true);
        m_enc->set_fps(30);
        addModule("VideoEncoder", "VideoEncoder_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), ModuleThread::loopFull);
        VideoEncoder *enc1 = addModule<VideoEncoder>("ENCODER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);
        enc1->set_fps(60);
        addModule("ImageViewer", "VIEWER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), ModuleThread::loopFull);
        VideoEncoder *enc3 = addModule<VideoEncoder>("ENCODER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);
        enc3->set_fps(60);
        addModule("ImageViewer", "VIEWER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_4", STRINGS("ueye_rgb_4"), ModuleThread::loopFull);
        VideoEncoder *enc4 = addModule<VideoEncoder>("ENCODER_4", STRINGS("ueye_rgb_4"), ModuleThread::listenFirst);
        enc4->set_fps(60);
        addModule("ImageViewer", "VIEWER_4", STRINGS("ueye_rgb_4"), ModuleThread::listenFirst);
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
