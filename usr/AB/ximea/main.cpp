// Sample for XIMEA Software Package V2.57
#include <xiApi.h>
#include <xiExt.h>

// docu:
// http://www.ximea.com/support/wiki/apis/XiApi_Manual

#include <MT/util.h>
#include <MT/array.h>
//#include <MT/opengl.h>
#include <MT/opencv.h>

#define HandleResult(res,place) if (res!=XI_OK) {printf("Error after %s (%d)",place,res);goto finish;}

int main(int argc, char** argv){
  // image buffer
  XI_IMG image;
  image.size = sizeof(XI_IMG);
  image.bp = NULL;
  image.bp_size = 0;

  ofstream file("video.raw", std::ios::out | std::ios::binary);
  //OpenGL gl;
  //gl.img = &img;

  // Sample for XIMEA API V2.10
  HANDLE xiH = NULL;
  XI_RETURN stat = XI_OK;

  // Get number of camera devices
  DWORD dwNumberOfDevices = 0;
  stat = xiGetNumberDevices(&dwNumberOfDevices);
  HandleResult(stat,"xiGetNumberDevices (no camera found)");

  if (!dwNumberOfDevices){
    printf("No camera found\n");
    goto finish;
  }

  stat = xiOpenDevice(0, &xiH);
  HandleResult(stat,"xiOpenDevice");

  stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, 10000);
  HandleResult(stat,"xiSetParam (exposure set)");

  stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
  HandleResult(stat,"xiSetParam (format)");

  // stat = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, 2);
  // HandleResult(stat,"xiSetParam (downsampling)");

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
  stat = xiStartAcquisition(xiH);
  HandleResult(stat,"xiStartAcquisition");

  for (int images=0; images<20 ;images++){
    // getting image from camera
    stat = xiGetImage(xiH, 5000, &image);
    HandleResult(stat,"xiGetImage");
    cout <<"image received: height=" <<image.height <<" width=" <<image.width <<" bp=" <<image.bp <<" bp_size=" <<image.bp_size <<" nframe=" <<image.nframe <<" format=" <<image.frm <<endl;

    //write to raw video
    if(image.frm==XI_MONO8) file.write((char*)image.bp, image.height*image.width);
    if(image.frm==XI_RAW8)  file.write((char*)image.bp, image.height*image.width);
    if(image.frm==XI_RGB24)  file.write((char*)image.bp, 3*image.height*image.width);

    //write single image
    if(images==10){
      byteA img;
      if(image.frm==XI_MONO8){
	img.referTo((byte*)image.bp,image.height*image.width);
	img.reshape(image.height,image.width);
	write_ppm(img, "pic.ppm");

	FILE * pFile; 
	pFile = fopen ( "pic.raw" , "wb" );
	fwrite (image.bp , 1 , image.height*image.width , pFile );  
	fclose (pFile);
      }
      if(image.frm==XI_RGB24){
	img.referTo((byte*)image.bp, 3*image.height*image.width);
	img.reshape(image.height, image.width, 3);
	write_ppm(img, "pic.ppm");

	FILE * pFile; 
	pFile = fopen ( "pic.raw" , "wb" );
	fwrite (image.bp , 1 , 3*image.height*image.width , pFile );  
	fclose (pFile);
      }

      if(image.frm=XI_RAW8){
        img.referTo((byte*)image.bp,image.height*image.width);
        img.reshape(image.height, image.width);
        byteA rgb(img.d0, img.d1, 3);
        cv::Mat dst=cvMAT(rgb);
        cv::Mat src=cvMAT(img);
        cv::cvtColor(src, dst, CV_BayerBG2RGB);

	FILE * pFile;
	pFile = fopen ( "pic.raw" , "wb" );
	fwrite (rgb.p , 1 , rgb.N, pFile );
	fclose (pFile);
      }
    }
//      img.referTo((byte*)image.bp,image.height*image.width);
//      img.reshape(image.height,image.width);
//      gl.update();
//    }
  }

finish:

  file.close();
  // Close device
  if(xiH) xiCloseDevice(xiH);
  printf("Done\n");
  return 0;
}
