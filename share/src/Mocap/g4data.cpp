#include <Core/array.h>
#include <Core/util.h>
#include <Core/geo.h>
#include <Core/graph.h>
#include <Core/registry.h>
#include <sys/stat.h>
#include "g4data.h"

G4Rec::G4Rec() { }

G4Rec::~G4Rec() { }

MocapRec *G4Rec::clone() {
  return static_cast<MocapRec*>(new G4Rec());
}

bool G4Rec::loadable(const char *recdir) {
  struct stat buffer;
  return (stat(STRING(recdir << "poses.dat"), &buffer) == 0);
}

void G4Rec::load(const char *recdir) {
  dir = STRING(recdir);
  mid.load(STRING(recdir << "meta.kvg"));

  ifstream datafin, tstampfin;
  MT::open(datafin, STRING(recdir << "poses.dat"));
  tstampfin.open(STRING(recdir << "poses.dat.times"));
  arr dataframe;

  nsensors = mid.sensors().N;

  bool m;
  boolA pm(nsensors);
  pm.setZero(false);
  uint currfnum;
  int hsi;
  double currtstamp, inittstamp = -1;
  arr data, tstamp, observed;
  boolA missing;
  MT::Array<intA> missingno(nsensors), missingf(nsensors);
  for(nframes = 0;; nframes++) {
    datafin >> dataframe;
    tstampfin >> currfnum >> currtstamp;
    if(inittstamp == -1)
      inittstamp = currtstamp;
    tstamp.append(currtstamp - inittstamp);
    // if(tstampfin.good()) {
    //   tstampfin >> currfnum >> currtstamp;
    //   tstamp.append(currtstamp);
    // }
    if(!dataframe.N || !datafin.good()) break;
    for(uint i = 0; i < nsensors; i++) {
      // find the row number for sensor number i
      hsi = mid.sensor_rn(mid.sensor(i));
      // cout << "hsi(" << mid.sensor(i) << "): " << hsi << endl;
      // hsi = mid.itohsi(i);
      if(hsi != -1) {
        data.append(dataframe[hsi]);
        m = (dataframe[hsi]().sparsity() == 0);
        missing.append(m);
        observed.append(!m);
        if(m && pm(i))
          missingno(i).last()++;
        else if(m && !pm(i)) {
          missingno(i).append(1);
          missingf(i).append(nframes);
        }
        pm(i) = m;
      }
    }
  }
  uint thinning = *params.get<uint>("thinning");
  nframes_thin = nframes / thinning;

  data.reshape(nframes, nsensors, 7);
  missing.reshape(nframes, nsensors);
  observed.reshape(nframes, nsensors);
  // for(uint f = 0; f < nframes; f++)
  //   for(uint s = 0; s < nsensors; s++)
  //     cout << f << ", " << s << ": (" << observed(f, s) << ") " << data[f][s] << endl;
  // exit(1);

  // setting quaternions as a continuous path on the 4d sphere
  arr dataquat, dataquatprev;
  dataquatprev = data[0].sub(0, -1, 3, -1);
  for(uint f = 1; f < data.d0; f++) {
    for(uint i = 0; i < data.d1; i++) {
      dataquat.referToSubRange(data.subDim(f, i)(), 3, -1);
      if(sum(dataquat % dataquatprev[i]) < 0)
        dataquat *= -1.;
      if(!length(dataquatprev[i]) || length(dataquat))
        dataquatprev[i]() = dataquat;
    }
  }
  // for(uint f = 0; f < nframes; f++)
  //   for(uint s = 0; s < nsensors; s++)
  //     cout << f << ", " << s << ": (" << observed(f, s) << ") " << data[f][s] << endl;
  // exit(1);

  String interpolate = *params.get<String>("interpolate");
  if(interpolate == "true") { // interpolating missing measures
    for(uint i = 0; i < nsensors; i++) {
      for(uint j = 0; j < missingno(i).N; j++) {
        uint t = missingf(i).elem(j);
        uint no = missingno(i).elem(j);
        if(t == 0 && no == nframes) // sensor is missing completely
          continue;
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            data[tt][i] = data[no][i];
        else if(t+no < nframes) { // interpolate between t-1 and t+missingno(i)
          arr s0 = data[t-1][i];
          arr sF = data[t+no][i];
          ors::Quaternion q0(s0(3), s0(4), s0(5), s0(6));
          ors::Quaternion qF(sF(3), sF(4), sF(5), sF(6));
          ors::Quaternion qt;

          arr diff = sF - s0;
          for(uint tt = 0; tt < no; tt++) {
            data[t+tt][i] = s0 + diff*(tt+1.)/(no+1.);
            qt.setInterpolate((tt+1.)/(no+1.), q0, qF);
            data(t+tt, i, 3) = qt.w;
            data(t+tt, i, 4) = qt.x;
            data(t+tt, i, 5) = qt.y;
            data(t+tt, i, 6) = qt.z;
          }
        }
        else // set all equal to last
          for(uint tt = 0; tt < no; tt++)
            data[nframes-tt-1][i] = data[nframes-no-1][i];
      }
    }
  }
  // for(uint f = 0; f < nframes; f++)
  //   for(uint s = 0; s < nsensors; s++)
  //     cout << f << ", " << s << ": (" << observed(f, s) << ") " << data[f][s] << endl;
  // exit(1);

  // setting up default BAMs
  arr pose, obs;
  tensorPermutation(pose, data, {1u, 0u, 2u});
  arr pos = pose.sub(0, -1, 0, -1, 0, 2);
  arr quat = pose.sub(0, -1, 0, -1, 3, -1);
  tensorPermutation(obs, observed, {1u, 0u});

  // organizing data about this dir
  appendBam("pos", pos);
  appendBam("quat", quat);
  appendBam("pose", pose);
  appendBam("posObs", obs);
  appendBam("quatObs", obs);
  appendBam("poseObs", obs);
  appendBam("tstamp", tstamp);

  // loading annotation, if any..
  Graph kvgtmp;
  try {
    FILE(STRING(recdir << "ann_kvg.kvg")) >> kvgtmp;
  }
  catch(const char *e) {
    cout << "No annotations in " << recdir << "." << endl;
  }
  uint from, to;

  arr *ann;
  for(Item *pair: kvgtmp) {
    ann = new arr(nframes);
    ann->setZero();
    if(!targets.contains(pair->keys(0)))
      targets.append(pair->keys(0));

    if(!agent_targets.getItem(pair->keys(0)))
      agent_targets.append((char*)pair->keys(0), new StringA());
    if(!object_targets.getItem(pair->keys(0)))
      object_targets.append((char*)pair->keys(0), new StringA());

    StringA &a_targets = *agent_targets.getValue<StringA>(pair->keys(0));
    StringA &o_targets = *object_targets.getValue<StringA>(pair->keys(0));

    if(!a_targets.contains(pair->keys(1)))
      a_targets.append(pair->keys(1));
    if(!o_targets.contains(pair->keys(2)))
      o_targets.append(pair->keys(2));

    for(Item *lock: *pair->getValue<Graph>()) {
      from = (uint)*lock->getValue<Graph>()->getValue<double>("from");
      to = (uint)*lock->getValue<Graph>()->getValue<double>("to");
      ann->subRange(from, to) = 1;
    }
    // pair->getValue<Graph>()->append("ann", ann);
    mlabel.append(pair->keys, {}, ann, false);
  }
}
