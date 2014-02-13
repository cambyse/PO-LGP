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

        VideoEncoderX264 *m_enc = addModule<VideoEncoderX264>("VideoEncoder_rgb", STRINGS("kinect_rgb"), ModuleThread::listenFirst);
        m_enc->set_rgb(true);
        addModule("VideoEncoderX264", "VideoEncoder_depth", STRINGS("kinect_depthRgb"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_1", STRINGS("ueye_rgb_1"), ModuleThread::loopFull);
        addModule("VideoEncoderX264", "ENCODER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);
        addModule("ImageViewer", "VIEWER_1", STRINGS("ueye_rgb_1"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_3", STRINGS("ueye_rgb_3"), ModuleThread::loopFull);
        addModule("VideoEncoderX264", "ENCODER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);
        addModule("ImageViewer", "VIEWER_3", STRINGS("ueye_rgb_3"), ModuleThread::listenFirst);

        addModule("UEyePoller", "POLLER_4", STRINGS("ueye_rgb_4"), ModuleThread::loopFull);
        addModule("VideoEncoderX264", "ENCODER_4", STRINGS("ueye_rgb_4"), ModuleThread::listenFirst);
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
