#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/video.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
#include "feedgnuplot.h"

#include <unistd.h>

void lib_hardware_G4();

void display(const G4Data &g4d) {
  //VideoEncoder_libav_simple vid;
  OpenGL gl;
  ors::Graph ors;

  MT::String shapes = MT::getParameter<MT::String>("shapes");
  init(ors, gl, shapes);
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();

  SwiftInterface swift;
  swift.init(ors, 2.);
  swift.computeProxies(ors);

  StringA posNames;
  StringA oriNames;

  // add entities to keyframer
  KeyFramer kf;
  for(String name: g4d.getNames()) {
    String pname = STRING(name << ":pos");
    posNames.append(pname);
    kf.addBody(pname, 3);

    String oname = STRING(name << ":ori");
    oriNames.append(oname);
    kf.addBody(oname, 4);;
  }

  uint T = g4d.getNumTimesteps();
  for(uint t = 0; t < T; t++) {
    kf.addState();
    for(uint i = 0; i < posNames.N; i++) {
      String name = g4d.getName(i);
      String pname = posNames(i);
      String oname = oriNames(i);

      kf.setState(pname, g4d.queryPos(t, name), t);
      kf.setState(oname, g4d.queryQuat(t, name), t);
    }
  }

  String b1("rh:thumb"), b2("sbox");
  String bp1(STRING(b1 << ":pos")), bp2(STRING(b2 << ":pos"));
  String bo1(STRING(b1 << ":ori")), bo2(STRING(b2 << ":ori"));

  uintA wlens = { 20, 30, 40 };
  uint mwlen = wlens.max();
  MT::Array<arr> corrPos = kf.getCorrEnsemble(bp1, bp2, wlens, true);

  uint wlen = 30;
  arr corrOri = kf.getAngle(bo1, bo2);
  arr corrVar = kf.getAngleVar(bo1, bo2, wlen);

  Feedgnuplot gnup;
  gnup.setDataID(true);
  gnup.setAutolegend(true);
  gnup.setStream(.75);
  gnup.open();

  MT::String bname;
  for(uint t = 0; t < T; t++) {
    for(auto &b: ors.bodies) {
      arr x = g4d.query(t, b->name);
      x.reshape(x.N); // TODO there should be a nicer way.. // resetD doesn't work
      CHECK(length(x)!=0, "Why isn't interpolation on?");

      b->X.pos.set(x(0), x(1), x(2));
      b->X.rot.set(x(3), x(4), x(5), x(6));
    }

    ors.calcBodyFramesFromJoints();
    //ors.calcShapeFramesFromBodies();

    uint bi1 = ors.getBodyByName(b1)->index;
    uint bi2 = ors.getBodyByName(b2)->index;
    swift.computeProxies(ors);
    //ors.reportProxies();
    //cout << "proxy: " << endl;
    //cout << ors.proxies << endl;
    for(ors::Proxy *p: ors.proxies) {
      cout << "Proxy:" << endl;
      cout << " - d: " << p->d << endl;
      cout << " - cenD: " << p->cenD << endl;
      cout << " - posA: " << p->posA << endl;
      cout << " - posB: " << p->posB << endl;
      cout << " - a->pos: " << ors.shapes(p->a)->body->X.pos << endl;
      cout << " - b->pos: " << ors.shapes(p->b)->body->X.pos << endl;
      cout << " - l_diff: " << (ors.shapes(p->a)->body->X.pos -
          ors.shapes(p->b)->body->X.pos).length() << endl; }
    
    gl.text.clear() <<"frame " <<t <<"/" <<T;
    gl.update(NULL, true);
    //flip_image(gl.captureImage);
    //vid.addFrame(gl.captureImage);

    String ss;
    ss << t << " ori " << -(corrOri(t)/(2*M_PI));
    if(t>=wlen) {
      uint wi = t-wlen;
      ss << "var " << corrVar(wi);
    }
    if(t>=mwlen) {
      uint wi = t-mwlen;

      double c = 1;
      for(uint wii = 0; wii < wlens.N; wii++)
        c *= corrPos(wii).elem(wi);
      ss << " pos " << c;
    }
    gnup() << ss;
  }
  usleep(1000000);

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

