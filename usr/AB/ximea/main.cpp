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
    XI_IMG image;
    HANDLE xiH;
    XI_RETURN stat;

    OpenGL gl;
    byteA img;
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
                ndev(0), xiH(NULL), stat(XI_OK) {
        image.size = sizeof(XI_IMG);
        image.bp = NULL;
        image.bp_size = 0;
    };

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
                break;
            case '-':
            case 13: // Enter
                make_snapshot();
                break;
        }
        return true;
    }

    void openAll() {
        openDir();
        if(!error) {
            openFile();
            openCredits();
            openCam();

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
        file.open("video.raw", std::ios::out | std::ios::binary);
    }

    void openCredits() {
        //read_ppm(displayImg,"box.ppm",false);
        //make_grey(displayImg);
        gl.img = &img;
        gl.update();
    }

    void openCam() {
        // Get number of camera devices
        stat = xiGetNumberDevices(&ndev);
        checkStatus("xiGetNumberDevices (no camera found)");

        if(!ndev) {
            cout << "No camera found" << endl;
            error = true;
            quit = true;
        }
        else {
            stat = xiOpenDevice(0, &xiH);
            checkStatus("xiOpenDevice");

            stat = xiSetParamInt(xiH, XI_PRM_EXPOSURE, 10000);
            checkStatus("xiSetParam (exposure set)");

            stat = xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
            checkStatus("xiSetParam (format)");

            stat = xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, 2);
            checkStatus("xiSetParam (downsampling)");

            float min_fps, max_fps;
            xiGetParamFloat(xiH, XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &min_fps);
            xiGetParamFloat(xiH, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &max_fps);
            cout <<"available frame rates = " << min_fps << '-' << max_fps << endl;

            int cfa;
            xiGetParamInt(xiH, XI_PRM_COLOR_FILTER_ARRAY, &cfa);
            cout << "buffer format = " << cfa << endl;

            int iscolor;
            xiGetParamInt(xiH, XI_PRM_IMAGE_IS_COLOR,&iscolor);
            cout << "color = " << iscolor << endl;
        }
    }

    void closeAll() {
        closeFile();
        closeCam();
    }

    void closeFile() {
        file.close();
    }

    void closeCam() {
        if(xiH)
            xiCloseDevice(xiH);
        cout << "Device closed." << endl;
    }

    void startCam() {
        // Start acquisition
        stat = xiStartAcquisition(xiH);
        checkStatus("xiStartAcquisition");
    }

    void getImage() {
        stat = xiGetImage(xiH, 5000, &image);
        checkStatus("xiGetImage");

        // Transforming to byteA
        if(image.frm == XI_MONO8) {
            img.referTo((byte*)image.bp, image.height*image.width);
            img.reshape(image.height, image.width);
        }
        else if(image.frm == XI_RAW8) {
            img.referTo((byte*)image.bp, image.height*image.width);
            img.reshape(image.height, image.width);
        }
        else if(image.frm == XI_RGB24) {
            img.referTo((byte*)image.bp, 3*image.height*image.width);
            img.reshape(image.height, image.width, 3);
        }
    }

    void playStream() {
        if(play)
            gl.update();
    }

    void recordStream() {
        if(rec) {
            //write to raw video
            if(image.frm == XI_MONO8)
                file.write((char*)image.bp, image.height*image.width);
            else if(image.frm == XI_RAW8)
                file.write((char*)image.bp, image.height*image.width);
            else if(image.frm == XI_RGB24)
                file.write((char*)image.bp, 3*image.height*image.width);
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

    void checkStatus(string place) {
        if(stat != XI_OK) {
            cout << "Error after " << place << " (" << stat << ")" << endl;
            closeFile();
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

    while(!cam.quit) {
        cam.getImage();
        cam.playStream();
        cam.recordStream();
    }
    cout << "Quitting." << endl;

    cout << "Closing All." << endl;
    cam.closeAll();

    cout << "Error = " << cam.error << endl;

    return 0;
}

