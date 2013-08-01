using namespace std;

Recorder::Recorder(): quit(false), play(true), rec(false) {
  gl.addKeyCall(this);

  numCams = UeyeCamera::getNumCameras();

  cameras = new UEyeCamera*[numCams];
  cameraThreads = new CameraThread*[numCams];

  for(int c = 0; c < numCams; c++)
    cameras[i] = new UeyeCamera(c, 1280, 1024, 60, "video.avi");

  for(int c = 0; c < numCams; c++)
    cameraThreads = new CameraThread(cameras[i]);

  /*
  if(kinect)
    kinectThread = new KinectThread("kinect.avi");
  */

  // Init GL
  img = new byteA*[numCams];
  for(int c = 0; c < numCams; c++)
    img[c] = new byteA(h, w, 3);

  int nr = (numCams-1) / MAX_CAMS_PER_ROW + 1;
  int nc = MAX_CAMS_PER_ROW;
  float r, c;
  for(int cam = 0; cam < numCams; cam++) {
    gl.addView(cam, &nothing, 0);
    r = cam / nc;
    c = cam % nc;
    gl.setViewPort(cam, c/nc, (c+1)/nc, r/nr, (r+1)/nr);
    gl.views(cam).img = img[cam];
  }

  connect(&times, SIGNAL(timeout()), this, SLOT(updateDisplay()));
}

Recorder::~Recorder() {
  for(int c = 0; c < numCams; c++) {
    delete cameraThreads[c];
    delete cameras[c];
    delete img[c];
  }

  delete[] cameraThreads;
  delete[] cameras;
  delete[] img;

  /*
  if(kinect)
    delete kinectThread;
  */
}

void Recorder::record() {
  initAll();
  openAll();
  captureAll();
  closeAll();
}

bool Recorder::keyCallback(OpenGL &) {
  switch(gl.pressedkey) {
    case 'p':
      pressPlay();
      break;
    case 'q':
      pressQuit();
      break;
    case 'r':
      pressRecord();
      break;
    default:
      cout << "Unknown key pressed: " << gl.pressedkey << endl;
      break;
  }
  return true;
}

void Recorder::pressPlay() {
  play = !play;
  if(play) {
    for(int c=0;c<n_cameras;c++)
      cameraThreads[c]->start();

    if (useKinect)
      kinectThread->start();

    //display frequency (does not affect recording)
    displayTimer.start(1000/30);
  }
}

void Recorder::pressRecord() {
  rec = !rec;
}

void Recorder::pressQuit() {
  quit = true;

  for(int c=0;c<n_cameras;c++)
    cameraThreads[c]->stop();

  if (useKinect)
    kinectThread->stop();

  _displayTimer.stop();
}

void Recorder::updateDisplay() {
  gl.update();
}

