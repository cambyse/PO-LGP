#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Perception/g4data.h>
#include <Perception/keyframer.h>
//#include <Perception/symbolizer.h>
#include "fgplot.h"
#include "changepoint.h"
#include <unistd.h>
#include <functional>
#include <math.h>
#include <Core/array.h>
#include <Core/util.h>

using namespace std;

bool orsDrawProxies;

void init_gl(OpenGL &gl) {
  gl.camera.setPosition(7., .5, 3.);
  gl.camera.focus(0, .5, .5);
  gl.camera.upright();
  orsDrawProxies = false;
}

double prior(uint t) {
  return .1;
}

double data_loglike(const arr &data, uint t, uint s) {
  int n = s - t;
  arr data_range;
  data_range.referToSubRange(data, t, s-1); // TODO check the range
  double mean = sum(data_range) / data_range.N;
  arr data_range_mean = data_range - mean;
  double muT = (n * mean) / (1 + n);
  arr data_range_muT = data_range - muT;
  double nuT = 1. + n;
  double alphaT = 1. + n / 2.;
  double betaT = 1. + .5 * sumOfSqr(data_range_mean) + (n/(1+n)) * mean * mean / 2;
  double scale = (betaT * (nuT + 1))/(alphaT * nuT);

  double prob = sum(log(1. + (data_range_muT%data_range_muT)/(nuT * scale)));
  double lgA = lgamma((nuT+1)/2.) - log(sqrt(M_PI * nuT * scale)) - lgamma(nuT/2.);
    
  return n * lgA - (nuT + 1.)/2. * prob;
}

void cptest() {
  uint n = 10;
  arr data(n);
  MT::Rnd rnd;
  for(uint i = 0; i < n; i++) {
    if(i > 70)
      data(i) = 5 + rnd.gauss();
    else if(i > 55)
      data(i) = 7 + rnd.gauss();
    else if(i > 15)
      data(i) = 1 + rnd.gauss();
    else if(i > 6)
      data(i) = 3 + rnd.gauss();
    else
      data(i) = 1 + rnd.gauss();
  }

  cout << "data: " << data << endl;
  ChangePoint_offline cp;
  cp.detect(data, &prior, &data_loglike);

  arr pcp = cp.pcp();
  cout << "pcp: " << pcp << endl;

  KeyValueGraph kvg, *plot;
  kvg.append("data", "data", new arr(data));
  kvg.append("data", "pcp", new arr(pcp));
  plot = new KeyValueGraph();
  plot->append("title", new String(STRING("Data")));
  plot->append("dataid", new bool(true));
  plot->append("autolegend", new bool(true));
  plot->append("stream", new double(1));
  //plot->append("dump", new bool(true));
  plot->append("data", new String(STRING("data")));
  plot->append("data", new String(STRING("pcp")));
  kvg.append("plot", plot);

  FGPlots fgp;
  fgp.open(kvg);
  for(uint i = 0; i < data.d0; i++) {
    fgp.step(i);
    //fgp.replot();
    //MT::wait(.5);
  }
  //fgp.replot();
  //fgp.exit();
  MT::wait(10);
}

int main(int argc, char **argv) {
  //cptest();
  //exit(0);

  MT::initCmdLine(argc, argv);

#ifndef MT_extern_SWIFT
  cout << "no SWIFT loaded" << endl;
  return 1;
#endif

  KeyFramer kf;
  KeyValueGraph kvg, *plot;
  arr vit, vit2;
  kvgL ctrls, deltas;
  //Symbolizer symb;
  FGPlots fgp;
  arr query;
  uint n, d;

  MT::String data, meta, poses, train, test, world;
  MT::getParameter(data, "data");
  data = NULL; // TODO skip saved files for now
  MT::getParameter(meta, "meta");
  MT::getParameter(poses, "poses");
  MT::getParameter(train, "train");
  MT::getParameter(test, "test");
  MT::getParameter(world, "world");

  kf.kw().init(world);

  String subj, obj;
  StringA subjs, objs;

  subj = "human";
  obj = "bbox:body";

  subjs.append(STRING("rh"));
  subjs.append(STRING("lh"));
  objs.append(STRING("bbox:lid"));
  objs.append(STRING("bbox:body"));
  objs.append(STRING("ball"));

  init_gl(kf.kw().gl());
  kf.g4d().load(data, meta, poses, true);
  kf.process(kvg, subjs, objs);
  kf.playScene(kvg, subjs);

  // TODO
  // kf.model() // returns the state space model
  // kf.model().train(nodes);
  // kf.model().infer(latent, obs);
  // kf.model().params();
  //
  // FIXME why not.. do these two, both.. using Variables, Factors, etc..
  // FIXME check out the FactorGraph too....
  // struct HMM {
    // struct Params {
    // }
  // }
 
  // struct CRF {
    // struct Params {
    // }
  // }

  //int mode;
  //MT::getParameter(mode, "mode", 0);
  //switch(mode) {
    //case 0: // display ctrl-seq in ors
      //init_gl(kf.kw().gl());
      //kf.g4d().load(data, meta, poses, true);
      //kf.process(kvg, subj, obj);
      //kf.playScene(kvg, subj);
      //break;
    //case 1: // display ctrl-seq + observation plots
      //init_gl(kf.kw().gl());
      //kf.g4d().load(data, meta, poses, true);
      //kf.EM_z_with_c(kvg, subj, obj); // TODO solve this
      //fgp.open(kvg);
      //for(uint f = 0; f < kf.g4d().numFrames(); f++) {
        //kf.updateOrs(f, true);
        //fgp.step(f);
      //}
      //break;
    //case 2: // display ctrl-seq for multi-agent setupk
      //init_gl(kf.kw().gl());
      //kf.g4d().load(data, meta, poses, true);
      //kf.getVitSeq(vit, subj, obj);
      //kf.vitLogicMachine(kvg, vit2, vit);

      //kvg.clear();
      //kvg.append("data", "vit", new arr(vit2));
      //plot = new KeyValueGraph();
      //plot->append("title", new String("Vit Logic Machine"));
      //plot->append("dataid", new bool(true));
      //plot->append("autolegend", new bool(true));
      //plot->append("stream", new double(.75));
      //plot->append("data", new String("vit"));
      //kvg.append("plot", plot);

      //fgp.open(kvg);
      //for(uint f = 0; f < kf.g4d().numFrames(); f++) {
        //kf.updateOrs(f, true);
        //fgp.step(f);
      //}
      //break;
    //case 3: // print extracted keyframes
      //kf.g4d().load(data, meta, poses, true);
      //kf.getCtrlSeq(ctrls, subj, obj);
      //cout << "# Sigma-Ctrl-Seq: " << ctrls.N << endl;
      //kf.getDeltaSeq(deltas, ctrls);
      //cout << "# Delta-Ctrl-Seq: " << deltas.N << endl;
      //break;
    //case 4: // print GMM of d-states
      ////kf.g4d().load(data, meta, poses, true);
      ////kf.getCtrlSeq(ctrls, subj, obj);
      ////kf.getDeltaSeq(deltas, ctrls);
      ////symb.trainOnDelta(deltas);
      //// TODO visualize these..
      ////cout << "pi: " << symb.gmm.pi << endl;
      ////cout << "mu: " << symb.gmm.mu endl;
      ////cout << "sigma: " << symb.gmm.sigma endl;
      //break;
    //case 5: // train/test GMM of d-states
      ////kf.g4d().load(data, meta, train, true);
      ////kf.getCtrlSeq(ctrls, subj, obj);
      ////kf.getDeltaSeq(deltas, ctrls);
      ////symb.trainOnDelta(deltas);
      ////// create test
      ////kf.g4d().load(data, meta, test, true);
      ////kf.getCtrlSeq(ctrls, subj, obj);
      ////kf.getDeltaSeq(deltas, ctrls);
      ////n = deltas.N;
      ////d = deltas(0)->getValue<arr>("f_dpos")->N;
      ////query.resize(n, d);
      ////for(uint i = 0; i < n; i++)
        ////query[i]() = *deltas(i)->getValue<arr>("f_dpos");
      ////cout << "test posterior: " << symb.gmm.P(query) << endl;
      //break;
    //default:
      //HALT("mode not available.");
  //}

  return 0;
}

