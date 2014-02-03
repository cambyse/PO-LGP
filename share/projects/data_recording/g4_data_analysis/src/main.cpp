#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/videoEncoder.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "feedgnuplot.h"

#include <unistd.h>

void lib_hardware_G4();

void init_display(ors::KinematicWorld &kw) {
  MT::String shapes = MT::getParameter<MT::String>("shapes");
  kw.init(shapes);
  //init(G, gl, shapes);
  kw.gl().camera.setPosition(7., .5, 3.);
  kw.gl().camera.focus(0, .5, .5);
  kw.gl().camera.upright();
}

void display(G4Data &g4d) {
  //VideoEncoder_libav_simple vid;
  ors::KinematicWorld kw;

  init_display(kw);

  KeyFramer kf(kw, g4d);

  String b1("rh:thumb"), b2("sbox");
  uint wlen = 121;

  uintA vit;
  kf.EM(vit, b1, b2, wlen);

#if 1
  // OK this is stupid, but cope with it
  String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));

  arr c = kf.getCorrPCA(bp1, bp2, wlen, 1).flatten();
  arr pD = kf.getPos(b1, b2);
  arr pAVar = kf.getStateVar(bp1, wlen);
  arr pBVar = kf.getStateVar(bp2, wlen);
  arr qD = kf.getQuat(bo1, bo2);
  arr qAVar = kf.getStateVar(bo1, wlen);
  arr qBVar = kf.getStateVar(bo2, wlen);
  arr pVar = kf.getPosVar(b1, b2, wlen);
  arr qVar = kf.getQuatVar(bo1, bo2, wlen);

  Feedgnuplot gnup1;
  gnup1.setDataID(true);
  gnup1.setAutolegend(true);
  gnup1.setStream(.75);
  gnup1.setTitle("Observations + Viterbi");
  gnup1.setYRange(-1.5, 1.5);
  gnup1.open();

  Feedgnuplot gnup2;
  gnup2.setDataID(true);
  gnup2.setAutolegend(true);
  gnup2.setStream(.75);
  gnup2.setTitle("Misc.");
  gnup2.open();
  
  MT::String bname;
  uint F = g4d.getNumFrames();
  for(uint f = 0; f < F; f++) {
    kf.updateOrs(f);
    //gl.update(NULL, true);

    //flip_image(gl.captureImage);
    //vid.addFrame(gl.captureImage);

    String ss1, ss2, ss3;
    ss1 << f
            << " vit " << vit(f)
            << " c " << c(f)
            << " qVar " << qVar(f)
            << " pVar " << pVar(f)
            ;
    ss2 << f
            << " pAVar " << pAVar(f)
            << " pBVar " << pBVar(f)
            << " qAVar " << qAVar(f)
            << " qBVar " << qBVar(f)
            ;
            ;
    gnup1() << ss1;
    gnup2() << ss2;
  }
#endif

  KeyFrameL kfs = kf.getKeyFrames(vit);
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

