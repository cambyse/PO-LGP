// Sample for XIMEA Software Package V2.57
#include<xiApi.h>
#include<xiExt.h>

// docu:
// http://www.ximea.com/support/wiki/apis/XiApi_Manual

// System calls
#include<cstdio>
#include<cerrno>
#include<ctime>
#include<sys/stat.h>
#include<sys/types.h>
#include<fstream>

// ML code
#include<MT/util.h>
#include<Core/array.h>
#include<MT/opengl.h>
#include<MT/opencv.h>
#include<MT/videoWriter.h>

#include<string>

// Default folder names
#define SESSION     "session"
#define REC(r)      "/rec_" << r
#define SNAPSHOTS   "/snapshots"
#define CAM(c)      "/cam_" << c
#define FRAME(f)    "_frame_" << f

using namespace std;

// Key Callback struct
struct xiRec : OpenGL::GLKeyCall {
    static const string error_codes[];
    static const int num_keypad_codes[];

    static const uint width = 1280;
    static const uint height = 1024;

    static const int SELECT_EXPOSURE    = 1;
    static const int SELECT_GAIN        = 2;

    string session_dir_name;
    bool debug;
    bool play, rec, snap;
    bool quit, error, *cam_error;
    bool show_nframe;
    int nrec, nframe, oframe;
    time_t ntime, otime;
    float fps;
    int maxband;
    string param_select;
    int cam_select;
    long long unsigned int mult;
    int sign;
    unsigned int *cam_exposure;
    float *cam_gain;

    // Sample for XIMEA API V2.10
    DWORD ndev;
    XI_IMG *image;
    HANDLE *xiH;
    XI_RETURN *stat;

    OpenGL gl;
    byteA *img;
    ofstream file;//("video.raw",std::ios::out|std::ios::binary);

    VideoWriter *vw;
    //VideoWriter vw2;

    // Also,
    // image.height     -> height
    // image.width      -> width
    // image.bp         -> pointer to data
    // image.bp_size    -> NOT SURE
    // image.nframe     -> number of frame
    // image.frm        -> format

    xiRec():    session_dir_name(SESSION),
                debug(false),
                play(true), rec(false), snap(false),
                quit(false), error(false), cam_error(NULL),
                show_nframe(false),
                nrec(-1), nframe(-1), oframe(-1),
                fps(0),
                param_select(XI_PRM_EXPOSURE), cam_select(-1),
                mult(1), sign(1),
                cam_exposure(NULL), cam_gain(NULL),
                ndev(0), image(NULL), xiH(NULL), stat(NULL),
                vw(NULL) {
        // key calls
        gl.addKeyCall(this);
    }

    ~xiRec() {
        delete[] image;
        delete[] xiH;
        delete[] stat;
        delete[] cam_error;
        delete[] cam_exposure;
        delete[] cam_gain;
        delete[] vw;
    }

    bool keyCallback(OpenGL &) {
        switch(gl.pressedkey) {
            case 'c': // Show selected camera
                cout << "===| Current camera = " << cam_select << endl;
                break;
            case 'f': // show frame numbers
                show_nframe = !show_nframe;
                if(!show_nframe)
                    clearViews();
                break;
            case 'm': // shows current multiplier
                cout << "===| Current multiplier = " << mult << endl;
                break;
            case 'p': // Play stream
                play = !play;
                break;
            case 'q': // Quit
            case 27:  // Esc
                quit = true;
                break;
            case 'r': // record
                rec = !rec;
                if(rec) {
                    cout << "===| Starting Rec.." << endl;
                    nrec++;
                    openRecDir();
                    if(!error)
                        openRecVids();
                }
                else {
                    cout << "===| Ending Rec.." << endl;
                    closeRecVids();
                }
                break;
            case 'w': // Save Recording
                break;
            case 'x': // Select Exposure
                param_select = XI_PRM_EXPOSURE;
                cout << "===| Selecting Exposure" << endl;
                break;
            case 122: // y
                      // Select Gain
                param_select = XI_PRM_GAIN;
                cout << "===| Selecting Gain" << endl;
                break;
            case 149: // numpad key codes
            case 150:
            case 151:
            case 152:
            case 153:
            case 154:
            case 155:
            case 156:
            case 157:
            case 158:
            case 159:
                mult = pow(10, num_keypad_codes[ gl.pressedkey - 149]);
                cout << "===| Current multiplier = " << mult << endl;
                break;
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                if(gl.pressedkey - '0' > (int)ndev)
                    cout << "===| Can not select cam " << (gl.pressedkey - '0' - 1 ) << endl;
                else {
                    cam_select = gl.pressedkey - '0' - 1;
                    cout << "===| Selected cam " << (gl.pressedkey - '0' - 1 ) << endl;
                }
                break;
            case 171: // 171 is +
            case 173: // 173 is -
                if(gl.pressedkey == 171)
                    sign = 1;
                else
                    sign = -1;

                if(!param_select.compare(XI_PRM_EXPOSURE)) {
                    if(cam_select == -1) {
                        for(uint d = 0; d < ndev; d++) {
                            if(sign == -1 && (int)cam_exposure[d] + sign*(int)mult < 0)
                                cam_exposure[d] = 0;
                            else if(sign == 1 && cam_exposure[d] + sign*mult > 1000000)
                                cam_exposure[d] = 1000000;
                            else
                                cam_exposure[d] += sign*mult;

                            cout << "===| Setting Exposure (cam " << d << ") " << cam_exposure[d] << endl;
                            xiSetParamInt(xiH[d], XI_PRM_EXPOSURE XI_PRMM_DIRECT_UPDATE, cam_exposure[d]);
                        }
                    }
                    else {
                        if(sign == -1 && (int)cam_exposure[cam_select] + sign*(int)mult < 0)
                            cam_exposure[cam_select] = 0;
                        else if(sign == 1 && cam_exposure[cam_select] + sign*mult > 1000000)
                            cam_exposure[cam_select] = 1000000;
                        else
                            cam_exposure[cam_select] += sign*mult;

                        cout << "===| Setting Exposure (cam " << cam_select << ") " << cam_exposure[cam_select] << endl;
                        xiSetParamInt(xiH[cam_select], XI_PRM_EXPOSURE XI_PRMM_DIRECT_UPDATE, cam_exposure[cam_select]);
                    }
                }
                else {
                    if(cam_select == -1) {
                        for(uint d = 0; d < ndev; d++) {
                            if(sign == -1 && cam_gain[d] + sign*(float)mult < 0)
                                cam_gain[d] = 0;
                            else if(sign == 1 && cam_gain[d] + sign*mult > 1000000)
                                cam_gain[d] = 1000000;
                            else
                                cam_gain[d] += sign*mult;

                            cout << "===| Setting Gain (cam " << d << ") " << cam_gain[d] << endl;
                            xiSetParamFloat(xiH[d], XI_PRM_GAIN XI_PRMM_DIRECT_UPDATE, cam_gain[d]);
                        }
                    }
                    else {
                        if(sign == -1 && cam_gain[cam_select] + sign*(float)mult < 0)
                            cam_gain[cam_select] = 0;
                        else if(sign == 1 && cam_gain[cam_select] + sign*mult > 1000000)
                            cam_gain[cam_select] = 1000000;
                        else
                            cam_gain[cam_select] += sign*mult;

                        cout << "===| Setting Gain(cam " << cam_select << ") " << cam_gain[cam_select] << endl;
                        xiSetParamFloat(xiH[cam_select], XI_PRM_GAIN XI_PRMM_DIRECT_UPDATE, cam_gain[cam_select]);
                    }
                    break;
                }
                break;
            case ' ': // Record video
            case 13:  // Enter
                      // Record snapshot
                cout << "===| Taking Snapshot" << endl;
                recordSnapshot();
                break;
            default: // Print the key code
                cout << "===| Key Pressed: " << gl.pressedkey << endl;
        }
        return true;
    }

    void openAll() {
        openSessionDir();
        openCam();
        openViews();
    }

    void openSessionDir() {
        string dir_name_mod = "_0";
        int dir_num = 0;

        // directory creation
        while(mkdir(STRING(session_dir_name << dir_name_mod), 0777) == -1)
            dir_name_mod = STRING("_" << (++dir_num));
        session_dir_name = STRING(session_dir_name << dir_name_mod);
        cout << "===| Session Directory Created: " << session_dir_name << endl;

        if(mkdir(STRING(session_dir_name << SNAPSHOTS), 0777) == -1) {
            cout << "===| Error during creation of directory \"" << STRING(session_dir_name << SNAPSHOTS) << "\":" << endl;
            cout << "===|  - " << strerror(errno) << endl;
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
        for(uint d = 0; d < ndev; d++)
            vw[d].open(image[d].width, image[d].height, STRING(session_dir_name << REC(nrec) << CAM(d) << ".avi"), fps); // fps doesn't seem to be important at all..
        //vw2.open(ndev*width, height, "video2.avi", fps);
    }

    void closeRecVids() {
        for(uint d = 0; d << ndev; d++)
            vw[d].close();
        //vw2.close();
    }

    static void nothing(void*) {
    }

    void openViews() {
        float fr = (float)1.0/(float)ndev;

        img = new byteA[ndev];
        for(uint d = 0; d < ndev; d++) {
            gl.addView(d,&nothing,0);
            gl.setViewPort(d,d*fr,(d+1)*fr,0,1);
            gl.views(d).img = &img[d];
        }
    }

    void openCam() {
        // Debug info
        if(debug) {
            cout << "===| Requiring Debug info." << endl;
            XI_RETURN debug_stat = xiSetParamInt(0, XI_PRM_DEBUG_LEVEL, XI_DL_TRACE);
            if(debug_stat != XI_OK) {
                cout << "===| Error code means: " << error_codes[debug_stat] << endl;
                error = true;
                quit = true;
            }
        }

        if(error)
            return;

        // Get number of camera devices
        cout << "Getting Number of Devices.." << endl;
        XI_RETURN ndev_stat = xiGetNumberDevices(&ndev);

        if(!ndev || ndev_stat != XI_OK) {
            cout << "===| No camera found" << endl;
            if(ndev_stat != XI_OK)
                cout << "===| Error code means: " << error_codes[ndev_stat] << endl;
            error = true;
            quit = true;
            return;
        }

        cout << "===| Cameras found: " << ndev << endl << endl;
        image = new XI_IMG[ndev];
        xiH = new HANDLE[ndev];
        stat = new XI_RETURN[ndev];
        cam_error = new bool[ndev];
        cam_exposure = new unsigned int[ndev];
        cam_gain = new float[ndev];
        vw = new VideoWriter[ndev];

        memset(xiH, 0, ndev*sizeof(HANDLE));
        fill_n(stat, ndev, XI_OK);
        memset(cam_error, false, ndev*sizeof(bool));
        fill_n(cam_exposure, ndev, 20000);
        memset(cam_gain, 0.0, ndev*sizeof(float));

        if(ndev > 1) {
            XI_RETURN autoband_stat = xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);
            if(autoband_stat != XI_OK) {
                cout << "===| Error disabling Auto_Bandwidth_Calculation." << endl;
                cout << "===| Error code means: " << error_codes[autoband_stat] << endl;
                error = true;
                quit = true;
                return;
            }
        }

        for(uint d = 0; d < ndev; d++) {
            cout << "===| Opening Cam " << d << endl;
            stat[d] = xiOpenDevice(d, &xiH[d]);
            checkStatus(d, "xiOpenDevice");
            checkHandle(d);

            cout << "===| Cam " << d << " opened successfully." << endl << endl;
        }

        if(error)
            return;

        cout << "===| Getting Available Bandwidth" << endl;
        XI_RETURN maxband_stat = xiGetParamInt(xiH[0], XI_PRM_AVAILABLE_BANDWIDTH, &maxband);
        if(maxband_stat != XI_OK) {
            cout << "===| Error getting Max Bandwidth." << endl;
            cout << "===| Error code means: " << error_codes[maxband_stat] << endl;
            error = true;
            quit = true;
            return;
        }
        cout << "===| available bandwidth = " << maxband << " Mbs" << endl << endl;

        for(uint d = 0; d < ndev; d++) {
            cout << "===| Setting / Getting Parameters Cam " << d << endl;
            
            if(cam_error[d])
                continue;

            int sn;
            cout << "===| Getting Serial Number." << endl;
            stat[d] = xiGetParamInt(xiH[d], XI_PRM_DEVICE_SN, &sn);
            checkStatus(d, "xiGetParam (device name)");
            cout << "===| Serial Number = " << sn << endl;

            if(cam_error[d])
                continue;

            cout << "===| Setting Exposure = " << cam_exposure[d] << endl;
            stat[d] = xiSetParamInt(xiH[d], XI_PRM_EXPOSURE, cam_exposure[d]);
            checkStatus(d, "xiSetParam (exposure set)");

            if(cam_error[d])
                continue;

            cout << "===| Setting Gain = " << cam_gain[d] << endl;
            stat[d] = xiSetParamFloat(xiH[d], XI_PRM_GAIN, cam_gain[d]);
            checkStatus(d, "xiSetParam (gain set)");

            if(cam_error[d])
                continue;

            cout << "===| Setting Image Format" << endl;
            stat[d] = xiSetParamInt(xiH[d], XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24);
            checkStatus(d, "xiSetParam (format)");

            if(cam_error[d])
                continue;

            float min_fps, max_fps;
            cout << "===| Getting min-max Fps" << endl;
            xiGetParamFloat(xiH[d], XI_PRM_FRAMERATE XI_PRM_INFO_MIN, &min_fps);
            xiGetParamFloat(xiH[d], XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &max_fps);
            cout << "===| available frame rates = " << min_fps << '-' << max_fps << endl;

            cout << "===| Setting individual Bandwidth to TOT_BANDWIDTH / NUM_CAMERAS = " << (maxband/ndev) << " Mbs" << endl;
            stat[d] = xiSetParamInt(xiH[d], XI_PRM_LIMIT_BANDWIDTH, maxband/ndev);

            if(cam_error[d])
                continue;

            image[d].size = sizeof(XI_IMG);
            image[d].bp = NULL;
            image[d].bp_size = 0;

            cout << endl;
        }

        if(!error)
            cout << "===| All Cameras (" << ndev << ") opened successfully." << endl << endl;;
    }

    void closeCam() {
        for(uint d = 0; d < ndev; d++) {
            if(xiH[d])
                xiCloseDevice(xiH[d]);
            cout << "===| Device " << d << " closed." << endl;
        }
    }

    void startCam() {
        // Start acquisition
        for(uint d = 0; d < ndev; d++) {
            stat[d] = xiStartAcquisition(xiH[d]);
            checkStatus(d, "xiStartAcquisition");
        }
        time(&ntime);
        time(&otime);
    }

    void stopCam() {
        // Stop acquisition
        for(uint d = 0; d < ndev; d++) {
            stat[d] = xiStopAcquisition(xiH[d]);
            checkStatus(d, "xiStopAcquisition");
        }
    }

    void getImage() {
        nframe++;

        time(&ntime);
        if(difftime(ntime, otime) >= 1) {
            /*
            cout << endl;
            cout << "===| diffframes = " << nframe - oframe << endl;
            cout << "===| ntime = " << ntime << endl;
            cout << "===| otime = " << otime << endl;
            cout << "===| difftime = " << difftime(ntime, otime) << endl;
            cout << "===| fps = " << (nframe - oframe) / difftime(ntime, otime) << endl;
            */
            fps = (fps + (nframe - oframe) / (difftime(ntime, otime))) / 2;
            oframe = nframe;
            otime = ntime;
        }

        for(uint d = 0; d < ndev; d++) {
            stat[d] = xiGetImage(xiH[d], 5000, &image[d]);
            checkStatus(d, "xiGetImage");

            if(cam_error[d])
                continue;

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
                    cout << "===| TODO what is RGB PLANAR?!" << endl;
                    break;
            }
        }
    }

    void playStream() {
        if(play)
            gl.update();

        if(show_nframe) {
            for(uint d = 0; d < ndev; d++) {
                cout << "===| nframe: " << nframe << endl;
                cout << "===| cam_nframe: " << image[d].nframe << endl;
                cout << "===| fps: " << fps << endl;
                gl.views(d).text.clear();
                gl.views(d).text << "nframe: " << nframe << "\r\n";
                gl.views(d).text << "cam_nframe: " << image[d].nframe << "\r\n";
                gl.views(d).text << "fps: " << fps << endl;
            }
        }
    }

    void recordStream() {
        if(rec) {
            for(uint d = 0; d < ndev; d++)
                vw[d].addFrame(img[d]);
            //vw2.addFrameFromOpengl(gl);

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
                        cout << "===| TODO what is RGB PLANAR?!" << endl;
                        break;
                }
            }
            */
        }
    }

    void clearViews() {
        for(uint d = 0; d < ndev; d++)
            gl.views(d).text.clear();
    }

    void recordSnapshot() {
        string file_name, file_name_mod = ".ppm";
        int file_num = -1;
        for(uint d = 0; d < ndev ; d++) {
            if(rec)
                file_name = STRING(session_dir_name << REC(nrec) << SNAPSHOTS << CAM(d) << FRAME(nframe) << ".ppm");
            else {
                while(true) {
                    file_name = STRING(session_dir_name << SNAPSHOTS << CAM(d) << FRAME(nframe) << file_name_mod);
                    ifstream ifile(file_name.c_str());
                    if(!ifile)
                        break;
                    file_name_mod = STRING("_" << (++file_num) << ".ppm");
                }
            }
            
            switch(image[d].frm) {
                case XI_MONO8:
                    write_ppm(img[d], file_name.c_str());

                    /*
                    FILE *pFile; 
                    pFile = fopen("pic.raw", "wb");
                    fwrite(image.bp, 1, image.height*image.width, pFile);  
                    fclose(pFile);
                    */
                    break;
                    /*
                case XI_RAW8:
                    byteA rgb(img.d0, img.d1, 3);
                    cv::Mat dst=conv_Arr2CvRef(rgb);
                    cv::Mat src=conv_Arr2CvRef(img);
                    cv::cvtColor(src, dst, CV_BayerBG2RGB);

                    FILE *pFile;
                    pFile = fopen(STRING("pic" << nframe << ".raw"), "wb");
                    fwrite(rgb.p, 1, rgb.N, pFile);
                    fclose(pFile);
                    break;
                    */
                case XI_RGB24:
                    write_ppm(img[d], file_name.c_str());
                    /*
                    FILE *pFile; 
                    pFile = fopen("pic.raw", "wb");
                    fwrite(image.bp, 1, 3*image.height*image.width, pFile);  
                    fclose(pFile);
                    */
                    break;
                default:
                    cout << "===| Not taking the snapshot!" << endl;
                    break;
            }
        }
    }

    void checkStatus(uint d, string place) {
        if(stat[d] != XI_OK) {
            cout << "===| Error after " << place << " (cam: " << d << ", stat: " << stat[d] << ")" << endl;
            cout << "===| Error code means: " << error_codes[stat[d]] << endl;
            //closeFile();
            //closeCam();
            cam_error[d] = true;
            error = true;
            quit = true;
        }
    }

    void checkHandle(uint d) {
        if(xiH[d] == INVALID_HANDLE_VALUE) {
            cout << "===| Invalid Handle Value for Cam " << d << endl;
            //closeCam();
            cam_error[d] = true;
            error = true;
            quit = true;
        }
    }

    string checkError() {
        string s;
        if(error) {
            s = STRING("===| Any Cam: Error." << endl);
            for(uint d = 0; d < ndev; d++)
                if(cam_error[d])
                    s = STRING(s<< "===| Cam " << d << ": Error." << endl);
        }
        else
            s = STRING("===| No Cam Error." << endl);
        return s;
    }
};

const string xiRec::error_codes[] = {
    "Function Call Succeeded",
    "Invalid Handle",
    "Register read error",
    "Register write error",
    "Freeing resource error",
    "Freeing channel error",
    "Freeing bandwidth error",
    "Read block error",
    "Write block error",
    "No image",
    "Timeout",
    "Invalid arguments supplied",
    "Not supported",
    "Attach buffers error",
    "Overlapped result",
    "Memory allocation error",
    "DDl context is NULL",
    "DDL context is non zero",
    "DDL context exists",
    "Too many devices connected",
    "Camera context error",
    "Unknown error",
    "Unknown hardware",
    "Invalid TM file",
    "Invalid TM tag",
    "Incomplete TM",
    "Bus reset error",
    "Not implemented",
    "Shading too bright",
    "Shading too dark",
    "Gain is too low",
    "Invalid bad pixel list",
    "Bad pixel list realloc error",
    "Invalid pixel list",
    "Invalid Flash File System",
    "Invalid profile",
    "Invalid calibration",
    "Invalid buffer",
    "Invalid data",
    "Timing generator is busy",
    "Wrong operation open/write/read/close",
    "Acquisition already started",
    "Old version of device driver installed to the system",
    "To get error code please call GetLastError function",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "BEWARE: UNKNOWN ERROR CODE",
    "Unknown parameter",
    "Wrong parameter value",
    "BEWARE: UNKNOWN ERROR CODE",
    "Wrong parameter type",
    "Wrong parameter size",
    "Input buffer too small",
    "Parameter info not supported",
    "Parameter info not supported",
    "Data dormat not supported",
    "Read only parameter"
};

const int xiRec::num_keypad_codes[] = {
    7, 4, 8, 6, 2, 9, 3, 1, 5, 0
};

int main(int argc, char** argv) {
    xiRec cam;

    if(argc > 1)
        cam.debug = true;
    
    cam.openAll();
    if(cam.error) {
        cout << "===| Error Opening Something." << endl;
        cam.closeCam();
        return -1;
    }

    cam.startCam();
    if(cam.error) {
        cout << "===| Error Starting Something." << endl;
        cam.closeCam();
        return -2;
    }

    while(!cam.quit) {
        cam.getImage();

        if(cam.error)
            continue;

        cam.playStream();

        if(cam.error)
            continue;

        cam.recordStream();
    }
    cout << "===| Quitting." << endl;

    cout << "===| Stopping All." << endl;
    cam.stopCam();

    cout << "===| Closing All." << endl;
    cam.closeCam();

    cout << cam.checkError();

    if(cam.error)
        return -3;

    return 0;
}

