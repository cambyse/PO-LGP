#include <Core/array.h>
#include <Hardware/kinect/kinect.h>

REGISTER_MODULE(KinectDepthPacking)

void KinectDepthPacking::open(){}
void KinectDepthPacking::close(){}

void KinectDepthPacking::step(){
    kinect_depth.readAccess();
    MT::Array<uint16_t> depth = kinect_depth();
    double tstamp = kinect_depth.tstamp();
    kinect_depth.deAccess();

    if(depth.N>0){
        byteA buffer(depth.N, 3);
        buffer.setZero();
        // CHECK: what is this for? was just an experiment?
#if 0
        for(uint i=0;i<depth.N;i++) memmove(buffer.p+1+3*i, depth.p+i, 2);
#else
        uint16_t d;
        byte *rgb;
        for(uint i=0;i<depth.N;i++){
            d=depth.p[i];
            rgb=buffer.p+3*i;
            rgb[0]=0xff&(d>>8);
            rgb[1]=0xff&(d>>4);
            rgb[2]=0xff&(d);
        }
#endif
        buffer.reshape(depth.d0, depth.d1, 3);
        kinect_depthRgb.writeAccess();
        kinect_depthRgb() = buffer;
        kinect_depthRgb.tstamp() = tstamp;
        kinect_depthRgb.deAccess();
    }
}
