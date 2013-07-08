// Sample for XIMEA Software Package V2.57
#include <xiApi.h>
#include <xiExt.h>

// docu:
// http://www.ximea.com/support/wiki/apis/XiApi_Manual

#include <MT/util.h>
#include <Core/array.h>
#include <MT/opengl.h>
#include <MT/opencv.h>

#define HandleResult(res)

#include <MT/module.h>
#include <biros/biros_module.h>

DECLARE_MODULE(XimeaCam);

struct XimeaCam:XimeaCam_Base{
  ACCESS(byteA, ximeaImage);

  XI_IMG image;
  HANDLE xiH;
  XimeaCam():xiH(NULL){
    image.size = sizeof(XI_IMG);
    image.bp = NULL;
    image.bp_size = 0;

    XI_RETURN res;

    DWORD dwNumberOfDevices = 0;
    res = xiGetNumberDevices(&dwNumberOfDevices);
    CHECK(res==XI_OK, "Error "<<res);
    CHECK(dwNumberOfDevices,"No camera found");

    res = xiOpenDevice(0, &xiH);
    CHECK(res==XI_OK, "Error "<<res);

    res = xiSetParamInt(xiH, XI_PRM_EXPOSURE, 10000);
    CHECK(res==XI_OK, "Error "<<res);

    res = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24);
    CHECK(res==XI_OK, "Error "<<res);

    // res = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, 2);
  //  CHECK(res==XI_OK, "Error "<<res);

    float min_fps,max_fps;
    xiGetParamFloat(xiH, XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &min_fps);
    xiGetParamFloat(xiH, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &max_fps);
    cout <<"available frame rates:" <<min_fps <<'-' <<max_fps <<endl;

    int cfa;
    xiGetParamInt(xiH, XI_PRM_COLOR_FILTER_ARRAY, &cfa);
    cout <<"buffer format=" <<cfa <<endl;

    int iscolor;
    xiGetParamInt(xiH, XI_PRM_IMAGE_IS_COLOR, &iscolor);
    cout <<"color=" <<iscolor <<endl;

    // Start acquisition
    res = xiStartAcquisition(xiH);
    CHECK(res==XI_OK, "Error "<<res);
  }

  ~XimeaCam(){
    if(xiH) xiCloseDevice(xiH);
  }

  void step(){
    XI_RETURN res = xiGetImage(xiH, 5000, &image);
    cout <<"image received: height=" <<image.height <<" width=" <<image.width <<" bp=" <<image.bp <<" bp_size=" <<image.bp_size <<" nframe=" <<image.nframe <<" format=" <<image.frm <<endl;
    CHECK(res==XI_OK, "Error "<<res);
    byteA img;
    if(image.frm==XI_MONO8){
      img.referTo((byte*)image.bp, image.height*image.width);
      img.reshape(image.height,image.width);
    }
    if(image.frm==XI_RGB24){
      img.referTo((byte*)image.bp, 3*image.height*image.width);
      img.reshape(image.height, image.width, 3);
    }
    if(image.frm==XI_RAW8){
      img.referTo((byte*)image.bp, image.height*image.width);
      img.reshape(image.height, image.width);
    }
    ximeaImage.set()=img;
  }

};

int main(int argc, char** argv){
  ofstream file("video.raw", std::ios::out | std::ios::binary);
  OpenGL gl;
  byteA img;
  gl.img = &img;

  Module_Process cam("XimeaCam");

  cam.threadOpen();
  cam.waitForIdle();
  cam.threadLoop();

  cout <<registry() <<endl;

  Variable *v = biros().getOrCreateVariable<Variable>("ximeaImage", NULL);

  for (int images=0; images<1000 ;images++){
    // getting image from camera
    v->waitForNextWriteAccess();
    v->readAccess(NULL);
    img = *(byteA*)v->data;
    v->deAccess(NULL);
    if(!img.N) continue;

    //write to raw video
    file.write((char*)img.p, img.N);

    //write single image
    if(!(images%10)){
      write_ppm(img, "pic.ppm");
      gl.update();
//        img.referTo((byte*)image.bp,image.height*image.width);
//        img.reshape(image.height, image.width);
//        byteA rgb(img.d0, img.d1, 3);
//        cv::Mat dst=cvMAT(rgb);
//        cv::Mat src=cvMAT(img);
//        cv::cvtColor(src, dst, CV_BayerBG2RGB);
//        write_ppm(rgb, "pic-RAW8.ppm");

    }
  }

  cam.threadClose();

  file.close();
  printf("Done\n");
  return 0;
}
