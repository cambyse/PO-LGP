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
  String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));

  arr corr;
  arr corrVar;
  uint wlen = 60;
  //uintA wlens = { wlen, wlen+10, wlen-10 };
  uintA wlens = { wlen };

  corr = kf.getCorrPCA(bp1, bp2, wlens(0), 1);
  corrVar = kf.getAngleVar(bo1, bo2, wlens(0));
  for(uint i = 1; i < wlens.N; i++) {
    corr = corr % kf.getCorrPCA(bp1, bp2, wlen, 1);
    corrVar += kf.getAngleVar(bo1, bo2, wlen);
  }
  corr.flatten();
  corrVar *= 1./wlens.N;

  arr corrXYZ = kf.getCorr(bp1, bp2, wlen);

  //arr dists = kf.getDists(b1, b2);

  // TODO:
  //  - MAP-em, so that you can also estimate the variance
  //  - try different observations:
  //    * 1-dim corr, 3-dim corr XYZ, etc
  //    * Angle Variance
  //    * Transformation variance (i.e. check whether the motion is rigid or
  //    not.)
  //    * Best thing would be to check IF there is motion, and check if the
  //    motion is rigid, oder?
  //
  //arr q = kf.EM(corrXYZ);
  arr q = kf.EM(corr, corrVar);

  corr.flatten();

#if 1
  Feedgnuplot gnup1, gnup2;
  gnup1.setDataID(true);
  gnup1.setAutolegend(true);
  gnup1.setStream(.75);
  gnup1.setTitle("1-dim PCA aligned");
  gnup1.open();
  gnup2.setDataID(true);
  gnup2.setAutolegend(true);
  gnup2.setStream(.75);
  gnup2.setTitle("3-dim X-, Y-, Z-axis aligned");
  gnup2.open();

  MT::String bname;
  uint F = g4d.getNumFrames();
  for(uint f = 0; f < F; f++) {
    kf.updateOrs(f);
    //gl.update(NULL, true);

    //flip_image(gl.captureImage);
    //vid.addFrame(gl.captureImage);

    String ss1, ss2;
    ss1 << f << " var " << sqrt(corrVar(f))
            << " corr " << corr(f)
            << " q " << q(f, 1)
            ;
    ss2 << f << " corrX " << corrXYZ(f, 0)
            << " corrY " << corrXYZ(f, 1)
            << " corrZ " << corrXYZ(f, 2)
            ;
    gnup1() << ss1;
    gnup2() << ss2;
  }
#endif

  //ProxyL proxies = kf.getProxies(b1, b2);
  //KeyFrameL kfs = kf.getKeyFrames(corr, proxies);
  KeyFrameL kfs = kf.getKeyFrames(q);
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

