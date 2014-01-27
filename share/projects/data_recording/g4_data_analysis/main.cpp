#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/videoEncoder.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "feedgnuplot.h"

#include <unistd.h>

void lib_hardware_G4();

void init_display(ors::KinematicWorld &G, OpenGL &gl) {
  MT::String shapes = MT::getParameter<MT::String>("shapes");
  init(G, gl, shapes);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();
}

void display(G4Data &g4d) {
  //VideoEncoder_libav_simple vid;
  OpenGL gl;
  ors::KinematicWorld G;

  init_display(G, gl);

  KeyFramer kf(G, gl, g4d);

  String b1("rh:thumb"), b2("sbox");
  String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));

  arr corrPos;
  arr corrVar;
  uint wlen = 30;
  uintA wlens = { wlen, wlen+10, wlen-10 };
  for(uint wlen: wlens) {
    if(corrPos.N == 0) {
      corrPos = kf.getCorr(bp1, bp2, wlen, true);
      corrVar = kf.getAngleVar(bo1, bo2, wlen);
      continue;
    }
    corrPos = elemWiseProd(corrPos, kf.getCorr(bp1, bp2, wlen, true));
    corrVar += kf.getAngleVar(bo1, bo2, wlen);
  }
  corrVar *= 1./wlens.N;

  arr dists = kf.getDists(b1, b2);

#if 0
  Feedgnuplot gnup;
  gnup.setDataID(true);
  gnup.setAutolegend(true);
  gnup.setStream(.75);
  gnup.open();

  arr corrPosDer = corrPos;
  for(uint i = corrPosDer.N-1; i > 0; i--)
    corrPosDer(i) -= corrPosDer(i-1);
  corrPosDer(0) = 0;

  arr test = corrPos;
  for(uint i = 0; i < test.N-wlen; i++)
    test(i) += test(i+wlen);
  test *= .5;

  MT::String bname;
  uint F = g4d.getNumFrames();
  for(uint f = 0; f < F; f++) {
    kf.updateOrs(f);
    gl.update(NULL, true);

    //flip_image(gl.captureImage);
    //vid.addFrame(gl.captureImage);

    String ss;
    ss << f << " var " << sqrt(corrVar(f))
            << " corr " << corrPos(f)
            //<< " der " << corrPosDer(f)
            //<< " test " << test(f)
            << " dist " << dists(f)
            //<< " thresh .9"
            ;
    gnup() << ss;
  }
#endif

  ProxyL proxies = kf.getProxies(b1, b2);
  KeyFrameL kfs = kf.getKeyFrames(corrPos, proxies);
  kf.clearProxies();
  kf.saveKeyFrameScreens(kfs, 30);
  cout << "Tot num keyframes: " << kfs.N << endl;

  //gl.watch();
  //vid.close();
}

void loadData(G4Data &g4d) {
  MT::String meta = MT::getParameter<MT::String>("meta");
  MT::String poses = MT::getParameter<MT::String>("poses");
  g4d.loadData(meta, poses, true);
}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  G4Data g4d;
  loadData(g4d);
  display(g4d);

  return 0;
}

