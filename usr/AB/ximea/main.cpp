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
#include<MT/videoWriter.h>

#include<string>

// Default folder names
#define DEFAULT     "default"
#define REC(r)      "/rec_" << r
#define SNAPSHOTS   "/snapshots"
#define CAM(c)      "/cam_" << c

using namespace std;

// Key Callback struct
struct xiRec : OpenGL::GLKeyCall {
    static const uint width = 1280;
    static const uint height = 1024;

    string session_dir_name;
    bool play, rec, snap;
    bool quit, error, *cam_error;
    bool show_nframe;
    int nrec, nframe;

    // TODO what was this for?
    bool open;

    // Sample for XIMEA API V2.10
    DWORD ndev;
    XI_IMG *image;
    HANDLE *xiH;
    XI_RETURN *stat;

    OpenGL gl;
    byteA *img;
    ofstream file;//("video.raw",std::ios::out|std::ios::binary);

    VideoWriter *v;
    VideoWriter v2;

    // Also,
    // image.height     -> height
    // image.width      -> width
    // image.bp         -> pointer to data
    // image.bp_size    -> NOT SURE
    // image.nframe     -> number of frame
    // image.frm        -> format

    xiRec():    session_dir_name(DEFAULT),
                play(true), rec(false), snap(false),
                quit(false), error(false), cam_error(NULL),
                show_nframe(false),
                nrec(-1), nframe(-1),
                open(false),
                ndev(0), image(NULL), xiH(NULL), stat(NULL) {};

    bool keyCallback(OpenGL &) {
        cout << "Key Pressed: " << gl.pressedkey << endl;
        switch(gl.pressedkey) {
            case 'e': // Snapshot
                break;
            case 'f': // show frame numbers
                show_nframe = !show_nframe;
                if(!show_nframe)
                    clearViews();
                break;
            case 'p': // Play stream
            case ' ':
                play = !play;
                break;
            case 'q': // Quit
            case 27:  // Esc
                quit = true;
                break;
            case 'r': // record
                rec = !rec;
                if(rec) {
                    cout << "Starting Rec.." << endl;
                    nrec++;
                    openRecDir();
                    if(!error)
                        openRecVids();
                }
                else {
                    cout << "Ending Rec.." << endl;
                    closeRecVids();
                }
                break;
            case 't': // test/text
                if(strcmp(gl.views(0).text,"ok") == 0)
                    gl.views(0).text = "";
                else
                    gl.views(0).text = "ok";
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
                recordSnapshot();
                break;
        }
        return true;
    }

    void openAll() {
        openSessionDir();
        openCam();
        openCredits();

        // key calls
        gl.addKeyCall(this);
    }

    void openSessionDir() {
        string dir_name_mod = "";
        int dir_num = 0;

        // directory creation
        while(mkdir(STRING(session_dir_name << dir_name_mod), 0777) == -1)
            dir_name_mod = STRING("_" << (++dir_num));
        session_dir_name = STRING(session_dir_name << dir_name_mod);
        cout << "===| Session Directory Created: " << session_dir_name << endl;

        if(mkdir(STRING(session_dir_name << SNAPSHOTS), 0777) == -1) {
            cout << "Error during creation of directory \"" << STRING(session_dir_name << SNAPSHOTS) << "\":" << endl;
            cout << " - " << strerror(errno) << endl;
            error = true;
            quit = true;
        }
    }

    void openRecDir() {
        // directory creation
        if(mkdir(STRING(session_dir_name << REC(nrec)), 0777) == -1) {
            cout << "===| Error during creation of directory \"" << STRING(session_dir_name << REC(nrec)) << "\":" << endl;
            cout << "===|  - " << strerror(errno) << endl;
            error = true;
            quit = true;
        }
        if(mkdir(STRING(session_dir_name << REC(nrec) << SNAPSHOTS), 0777) == -1) {
            cout << "===| Error during creation of directory \"" << STRING(session_dir_name << REC(nrec) << SNAPSHOTS) << "\":" << endl;
            cout << "===|  - " << strerror(errno) << endl;
            error = true;
            quit = true;
        }
    }

    void openRecVids() {
        v = new VideoWriter[ndev];
        for(uint d = 0; d < ndev; d++)
            v[d].open(image[d].width, image[d].height, STRING(session_dir_name << REC(nrec) << CAM(d) << ".avi"), 20);
        //v2.open(2*width,height, "video2.avi", 30);
    }

    void closeRecVids() {
        /*
        for(uint d = 0; d << ndev; d++)
            v[d].close();
        */
        delete[] v;
        //v.close();
        //v2.close();
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
            cout << "===| No camera found" << endl;
            error = true;
            quit = true;
        }
        else {
            cout << "===| Cameras found: " << ndev << endl;
            image = new XI_IMG[ndev];
            xiH = new HANDLE[ndev];
            stat = new XI_RETURN[ndev];
            cam_error = new bool[ndev];

            memset(xiH, 0, ndev*sizeof(HANDLE));
            memset(stat, XI_OK, ndev*sizeof(XI_RETURN));
            memset(cam_error, false, ndev*sizeof(bool));

            for(uint d = 0; d < ndev; d++) {
                // add d parameter to checkStatus
                stat[d] = xiOpenDevice(d, &xiH[d]);
                checkStatus(d, "xiOpenDevice");

                stat[d] = xiSetParamInt(xiH[d], XI_PRM_EXPOSURE, 10000);
                checkStatus(d, "xiSetParam (exposure set)");

                stat[d] = xiSetParamInt(xiH[d], XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24);
                checkStatus(d, "xiSetParam (format)");

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
                cout << "===| Cam " << d << " opened." << endl;
            }
            cout << "===| All Cameras (" << ndev << ") opened." << endl;
        }
    }

    void closeAll() {
        closeCam();
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
        nframe++;
        for(uint d = 0; d < ndev; d++) {
            stat[d] = xiGetImage(xiH[d], 5000, &image[d]);
            checkStatus(d, "xiGetImage");

            // Transforming to byteA
            switch(image[d].frm) {
                case XI_MONO8:
                case XI_RAW8:
                    img[d].referTo((byte*)image[d].bp, image[d].height*image[d].width);
                    img[d].reshape(image[d].height, image[d].width);
                    break;
                case XI_MONO16:
                case XI_RAW16:
                    img[d].referTo((byte*)image[d].bp, 2*image[d].height*image[d].width);
                    img[d].reshape(image[d].height, image[d].width, 2);
                    break;
                case XI_RGB24:
                    img[d].referTo((byte*)image[d].bp, 3*image[d].height*image[d].width);
                    img[d].reshape(image[d].height, image[d].width, 3);
                    break;
                case XI_RGB32:
                    img[d].referTo((byte*)image[d].bp, 4*image[d].height*image[d].width);
                    img[d].reshape(image[d].height, image[d].width, 4);
                    break;
                case XI_RGB_PLANAR:
                    cout << "TODO what is RGB PLANAR?!" << endl;
                    break;
            }
            /*
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
            */
        }
    }

    void playStream() {
        if(play)
            gl.update();
        if(show_nframe) {
            for(uint d = 0; d < ndev; d++) {
                gl.views(d).text << "nframe: " << nframe << endl;
                gl.views(d).text << "cam_nframe: " << image[d].nframe << endl;
            }
        }
    }

    void recordStream() {
        if(rec) {
            for(uint d = 0; d < ndev; d++)
                v[d].addFrame(img[d]);
            //v2.addFrameFromOpengl(gl);

            /*
            for(uint d = 0; d < ndev; d++) {
                //write to raw video
                switch(image[d].frm) {
                    case XI_MONO8:
                    case XI_RAW8:
                        file.write((char*)image[d].bp, image[d].height*image[d].width);
                        break;
                    case XI_MONO16:
                    case XI_RAW16:
                        file.write((char*)image[d].bp, 2*image[d].height*image[d].width);
                        break;
                    case XI_RGB24:
                        file.write((char*)image[d].bp, 3*image[d].height*image[d].width);
                        break;
                    case XI_RGB32:
                        file.write((char*)image[d].bp, 4*image[d].height*image[d].width);
                        break;
                    case XI_RGB_PLANAR:
                        cout << "TODO what is RGB PLANAR?!" << endl;
                        break;
                }
            }
            */
        }
    }

    void clearViews() {
        for(int d = 0; d < ndev; d++)
            gl.views(d).text.clear();
    }

    void recordSnapshot() {
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
            cam_error[d] = true;
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
        cam.session_dir_name = DEFAULT;
    else
        cam.session_dir_name = string(argv[1]);
    
    cam.openAll();
    if(cam.error) {
        cout << "===| Error Opening Something." << endl;
        exit(-1);
    }

    cam.startCam();
    cam.getImage();
    cout << "===| cam.image[0].width = " << cam.image[0].width << endl;
    cout << "===| cam.image[0].height = " << cam.image[0].height << endl;
    while(!cam.quit) {
        cam.getImage();
        cam.playStream();
        cam.recordStream();
    }
    cout << "===| Quitting." << endl;

    cout << "===| Closing All." << endl;
    cam.closeAll();

    if(cam.error) {
        cout << "===| Any Cam: Error." << endl;
        for(uint d = 0; d < cam.ndev; d++)
            if(cam.cam_error[d])
                cout << "===| Cam " << d << ": Error." << endl;
    }
    else
        cout << "===| No Cam Error." << endl;

    return 0;
}

