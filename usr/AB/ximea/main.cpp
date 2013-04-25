// Sample for XIMEA Software Package V2.57
#include<xiApi.h>
#include<xiExt.h>

// docu:
// http://www.ximea.com/support/wiki/apis/XiApi_Manual

// System calls
#include<cstdio>
#include<cerrno>
#include<sys/stat.h>
#include<sys/types.h>

// ML code
#include<MT/util.h>
#include<MT/array.h>
#include<MT/opengl.h>
#include<MT/opencv.h>

#include<string>

// Default folder names
#define DEFAULT "default"
#define SNAP    "snap"

using namespace std;

// Key Callback struct
struct xiRec : OpenGL::GLKeyCall {
    string dir_name;
    bool play, rec, snap;
    bool quit, error;

    // what was this for?
    bool open;

    // Sample for XIMEA API V2.10
    DWORD ndev;
    XI_IMG *image;
    HANDLE *xiH;
    XI_RETURN *stat;

    OpenGL gl;
    byteA *img;
    ofstream file;//("video.raw",std::ios::out|std::ios::binary);

    // Also,
    // image.height     -> height
    // image.width      -> width
    // image.bp         -> NOT SURE
    // image.bp_size    -> NOT SURE
    // image.nframe     -> number of frame
    // image.frm        -> format

    xiRec():    dir_name(DEFAULT),
                play(true), rec(false), snap(false),
                quit(false), error(false),
                open(false),
                //ndev(0), xiH(NULL), stat(XI_OK) {
                ndev(0), image(NULL), xiH(NULL), stat(NULL) {};

    bool keyCallback(OpenGL &) {
        cout << "Key Pressed: " << gl.pressedkey << endl;
        switch(gl.pressedkey) {
            case 'e': // Snapshot
                break;
            case 'p': // Play stream
                play = !play;
                break;
            case 'q': // Quit
            case 27:  // Esc
                quit = true;
                break;
            case 'w': // Save Recording
                break;
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                break;
            case '+':
                cout << "Pressed +." << endl;
                gl.resize(100,50);
                gl.update();
                break;
            case '-':
                cout << "Pressed -." << endl;
                gl.resize(100,100);
                gl.update();
                break;
            case 13: // Enter
                make_snapshot();
                break;
        }
        return true;
    }

    void openAll() {
        openDir();
        if(!error) {
            openCam();
            openCredits();
            //openFile();

            // key calls
            gl.addKeyCall(this);
        }
    }

    void openDir() {
        string dir_name_mod = "";
        int dir_num = 0;

        // directory creation
        while(mkdir(STRING(dir_name << dir_name_mod), 0777) == -1)
            dir_name_mod = STRING("_" << (++dir_num));
        dir_name = STRING(dir_name << dir_name_mod);
        cout << "Directory Created: " << dir_name << endl;

        if(mkdir(STRING(dir_name << "/" << SNAP), 0777) == -1) {
            cout << "Error during creation of directory \"" << STRING(dir_name << "/" << SNAP) << "\":" << endl;
            cout << " - " << strerror(errno) << endl;
            error = true;
            quit = true;
        }
    }

    void openFile() {
        //file.open("video.raw", std::ios::out | std::ios::binary);
    }

    static void nothing(void*) {
    }

    void openCredits() {
        /*
        read_ppm(img,"signal.ppm",false);
        gl.img = &img;
        gl.update();

        gl.addView(0,&nothing,0);
        gl.addView(1,&nothing,0);

        gl.setViewPort(0,0,1,0,1);
        gl.setViewPort(0,.5,1,0,1);

        gl.views(0).img = &img;
        gl.views(1).img = &img;
        */
        
        float fr = (float)1.0/(float)ndev;

        img = new byteA[ndev];
        for(uint d = 0; d < ndev; d++) {
            read_ppm(img[d],"signal.ppm",false);
            gl.addView(d,&nothing,0);
            gl.setViewPort(d,d*fr,(d+1)*fr,0,1);
            gl.views(d).img = &img[d];
        }
    }

    void openCam() {
        // Get number of camera devices
        //stat = xiGetNumberDevices(&ndev);
        //checkStatus("xiGetNumberDevices (no camera found)");
        xiGetNumberDevices(&ndev);

        if(!ndev) {
            cout << "No camera found" << endl;
            error = true;
            quit = true;
        }
        else {
            cout << "Cameras found: " << ndev << endl;
            image = new XI_IMG[ndev];
            xiH = new HANDLE[ndev];
            stat = new XI_RETURN[ndev];
            
            for(uint d = 0; d < ndev; d++) {
                // add d parameter to checkStatus
                stat[d] = xiOpenDevice(d, &xiH[d]);
                checkStatus(d, "xiOpenDevice");

                stat[d] = xiSetParamInt(xiH[d], XI_PRM_EXPOSURE, 10000);
                checkStatus(d, "xiSetParam (exposure set)");

                stat[d] = xiSetParamInt(xiH[d], XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
                checkStatus(d, "xiSetParam (format)");

                /*
                stat[d] = xiSetParamInt(xiH[d], XI_PRM_DOWNSAMPLING, 2);
                checkStatus(d, "xiSetParam (downsampling)");
                */

                float min_fps, max_fps;
                xiGetParamFloat(xiH[d], XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &min_fps);
                xiGetParamFloat(xiH[d], XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &max_fps);
                cout <<"available frame rates = " << min_fps << '-' << max_fps << endl;

                int cfa;
                xiGetParamInt(xiH[d], XI_PRM_COLOR_FILTER_ARRAY, &cfa);
                cout << "buffer format = " << cfa << endl;

                int iscolor;
                xiGetParamInt(xiH[d], XI_PRM_IMAGE_IS_COLOR, &iscolor);
                cout << "color = " << iscolor << endl;

                image[d].size = sizeof(XI_IMG);
                image[d].bp = NULL;
                image[d].bp_size = 0;

                // Log message
                cout << "Cam " << d << " opened." << endl;
            }
            cout << "All Cameras (" << ndev << ") opened." << endl;
        }
    }

    void closeAll() {
        //closeFile();
        closeCam();
    }

    void closeFile() {
        file.close();
    }

    void closeCam() {
        for(uint d = 0; d < ndev; d++) {
            if(xiH[d])
                xiCloseDevice(xiH[d]);
            cout << "Device " << d << " closed." << endl;
        }
        delete[] image;
        delete[] xiH;
        delete[] stat;
    }

    void startCam() {
        // Start acquisition
        for(uint d = 0; d < ndev; d++) {
            stat[d] = xiStartAcquisition(xiH[d]);
            checkStatus(d, "xiStartAcquisition");
        }
    }

    void getImage() {
        for(uint d = 0; d < ndev; d++) {
            stat[d] = xiGetImage(xiH[d], 5000, &image[d]);
            checkStatus(d, "xiGetImage");

            // Transforming to byteA
            if(image[d].frm == XI_MONO8) {
                img[d].referTo((byte*)image[d].bp, image[d].height*image[d].width);
                img[d].reshape(image[d].height, image[d].width);
            }
            else if(image[d].frm == XI_RAW8) {
                img[d].referTo((byte*)image[d].bp, image[d].height*image[d].width);
                img[d].reshape(image[d].height, image[d].width);
            }
            else if(image[d].frm == XI_RGB24) {
                img[d].referTo((byte*)image[d].bp, 3*image[d].height*image[d].width);
                img[d].reshape(image[d].height, image[d].width, 3);
            }
        }
    }

    void playStream() {
        if(play)
            gl.update();
    }

    void recordStream() {
        if(rec) {
            for(uint d = 0; d < ndev; d++) {
                //write to raw video
                if(image[d].frm == XI_MONO8)
                    file.write((char*)image[d].bp, image[d].height*image[d].width);
                else if(image[d].frm == XI_RAW8)
                    file.write((char*)image[d].bp, image[d].height*image[d].width);
                else if(image[d].frm == XI_RGB24)
                     file.write((char*)image[d].bp, 3*image[d].height*image[d].width);
            }
        }
    }

    void make_snapshot() {
        /*
        //return STRING("snap_"<<frame_num);
        if(cam.snapshot) {
            if(image.frm == XI_MONO8) {
                write_ppm(img,STRING("pic"<<fr_num<<".ppm"));

                FILE *pFile; 
                pFile = fopen("pic.raw","wb");
                fwrite(image.bp,1,image.height*image.width,pFile);  
                fclose(pFile);
            }
            if(image.frm == XI_RAW8) {
                byteA rgb(img.d0,img.d1,3);
                cv::Mat dst=cvMAT(rgb);
                cv::Mat src=cvMAT(img);
                cv::cvtColor(src,dst,CV_BayerBG2RGB);

                FILE *pFile;
                pFile = fopen(STRING("pic"<<fr_num<<".raw"),"wb");
                fwrite(rgb.p,1,rgb.N,pFile);
                fclose(pFile);
            }
            if(image.frm == XI_RGB24) {
                write_ppm(img,STRING("pic"<<fr_num<<".ppm"));

                FILE *pFile; 
                pFile = fopen("pic.raw","wb");
                fwrite(image.bp,1,3*image.height*image.width,pFile);  
                fclose(pFile);
            }
            cam.snapshot = false;
        }
        */
    }

    void checkStatus(int d, string place) {
        if(stat[d] != XI_OK) {
            cout << "Error after " << place << " (" << stat << ")" << endl;
            //closeFile();
            closeCam();
            error = true;
            quit = true;
        }
    }

    bool quitOrError() {
        return quit || error;
    }
};

int main(int argc, char** argv){
    xiRec cam;

    if(argc != 2)
        cam.dir_name = DEFAULT;
    else
        cam.dir_name = string(argv[1]);
    
    cam.openAll();
    if(!cam.error)
        cam.startCam();

    cam.getImage();
    cout << "cam.image[0].width = " << cam.image[0].width << endl;
    cout << "cam.image[0].height = " << cam.image[0].height << endl;
    while(!cam.quit) {
        cam.getImage();
        cam.playStream();
        //cam.recordStream();
    }
    cout << "Quitting." << endl;

    cout << "Closing All." << endl;
    cam.closeAll();

    // TODO make error dynamic array
    cout << "Error = " << cam.error << endl;

    return 0;
}

