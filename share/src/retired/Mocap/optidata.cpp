#include <Core/array.h>
#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>
#include <sys/stat.h>
#include <json/json.h>
#include "optidata.h"

OptiRec::OptiRec() { setDefaultParams(); }
OptiRec::~OptiRec() { }

MocapRec *OptiRec::clone() {
  return static_cast<MocapRec*>(new OptiRec());
}

bool OptiRec::loadable(const char *recdir) {
  struct stat buffer;
  return (stat(STRING(recdir << "dataset.json"), &buffer) == 0);
}

void OptiRec::load(const char *recdir) {
  dir = STRING(recdir);
  // mid.load(STRING(recdir << "meta.kvg"));

  mid.load_json(STRING(recdir << "../scene.json"));

  ifstream datafin;
  Json::Value root, scene;
  Json::Reader reader;

  mlr::open(datafin, STRING(recdir << "../scene.json"));
  CHECK(reader.parse(datafin, root), "Failed to parse configuration: " << reader.getFormattedErrorMessages());
  datafin.close();

  // TODO do stuff
  mlr::open(datafin, STRING(recdir << "dataset.json"));
  CHECK(reader.parse(datafin, root), "Failed to parse configuration: " << reader.getFormattedErrorMessages());
  datafin.close();

  nsensors = mid.sensors().N;

  bool m;
  boolA pm(nsensors);
  pm.setZero(false);
  // uint currfnum;
  // int hsi;
  // double currtstamp;
  arr data, tstamp, observed;
  boolA missing;
  mlr::Array<intA> missingno(nsensors), missingf(nsensors);

  const StringA &sensors = mid.sensors();
  // scene["objects"].getMemberNames();
  nframes = root["data"].size();
  uint thinning = *params.get<uint>("thinning");
  nframes_thin = nframes / thinning;

  for(uint f = 0; f < nframes; f++) {
    const Json::Value &frame = root["data"][f];
    tstamp.append(frame["time_sec"].asDouble());

    // TODO append stuff, and then set it according to the indices in ID.
    for(uint is = 0; is < nsensors; is++) {
      // const std::string &sensor = sensors[is];
      const String &sensor = sensors(is);
      m = !frame.isMember(sensor);
      missing.append(m);
      observed.append(!m);
      if(m)
        data.append({0., 0., 0., 0., 0., 0., 0.});
      else {
        mlr::Transformation T;
        T.setZero();
        T.addRelativeRotationDeg(90, 1, 0, 0);
        T.addRelativeTranslation(
                            mlr::Vector(frame[sensor]["pose"][0][0].asDouble(),
                                        frame[sensor]["pose"][0][1].asDouble(),
                                        frame[sensor]["pose"][0][2].asDouble()));
        T.addRelativeRotation(
                        mlr::Quaternion(frame[sensor]["pose"][1][3].asDouble(),
                                        frame[sensor]["pose"][1][0].asDouble(),
                                        frame[sensor]["pose"][1][1].asDouble(),
                                        frame[sensor]["pose"][1][2].asDouble()));
        // T.pos.set(frame[sensor]["pose"][0][0].asDouble(),
        //           frame[sensor]["pose"][0][1].asDouble(),
        //           frame[sensor]["pose"][0][2].asDouble());
        // T.rot.set(frame[sensor]["pose"][1][3].asDouble(),
        //           frame[sensor]["pose"][1][0].asDouble(),
        //           frame[sensor]["pose"][1][1].asDouble(),
        //           frame[sensor]["pose"][1][2].asDouble());
        data.append(T.pos.x);
        data.append(T.pos.y);
        data.append(T.pos.z);
        data.append(T.rot.w);
        data.append(T.rot.x);
        data.append(T.rot.y);
        data.append(T.rot.z);

        // data.append(frame[sensor]["pose"][0][0].asDouble());
        // data.append(frame[sensor]["pose"][0][1].asDouble());
        // data.append(frame[sensor]["pose"][0][2].asDouble());
        // data.append(frame[sensor]["pose"][1][3].asDouble());
        // data.append(frame[sensor]["pose"][1][0].asDouble());
        // data.append(frame[sensor]["pose"][1][1].asDouble());
        // data.append(frame[sensor]["pose"][1][2].asDouble());
      }
      if(m && pm(is)) {
        missingno(is).last()++;
      }
      else if(m && !pm(is)) {
        missingno(is).append(1);
        missingf(is).append(f);
      }
    }
  }
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
      dataquat.referToRange(data.refDim(f, i)(), 3, -1);
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
        if(t == 0) // set all equal to first
          for(uint tt = 0; tt < no; tt++)
            data[tt][i] = data[no][i];
        else if(t+no < nframes) { // interpolate between t-1 and t+missingno(i)
          arr s0 = data[t-1][i];
          arr sF = data[t+no][i];
          mlr::Quaternion q0(s0(3), s0(4), s0(5), s0(6));
          mlr::Quaternion qF(sF(3), sF(4), sF(5), sF(6));
          mlr::Quaternion qt;

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
  try {
    FILE(STRING(recdir << "ann_kvg.kvg")) >> kvgann;
  }
  catch(const char *e) {
    cout << "No annotations in " << recdir << "." << endl;
  }
  uint from, to;

  arr *ann;
  for(Node *pair: kvgann) {
    ann = new arr(nframes);
    ann->setZero();
    if(!targets.contains(pair->keys(0)))
      targets.append(pair->keys(0));

    if(!agent_targets.getNode(pair->keys(0)))
      // agent_targets.append(pair->keys(0), new StringA());
      agent_targets.newNode({pair->keys(0)}, {}, StringA());
    if(!object_targets.getNode(pair->keys(0)))
      object_targets.newNode({pair->keys(0)}, {}, StringA());
    StringA &a_targets = agent_targets.get<StringA>(pair->keys(0));
    StringA &o_targets = object_targets.get<StringA>(pair->keys(0));
    if(!a_targets.contains(pair->keys(1)))
      a_targets.append(pair->keys(1));
    if(!o_targets.contains(pair->keys(2)))
      o_targets.append(pair->keys(2));
    for(Node *lock: pair->graph()) {
      from = (uint)lock->graph().get<double>("from");
      to = (uint)lock->graph().get<double>("to");
      ann->refRange(from, to) = 1;
    }
    pair->graph().newNode<arr*>({"ann"}, {}, ann);
  }
}

