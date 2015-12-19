#include <Core/array.h>
#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>
#include <Core/registry.h>
#include <sys/stat.h>
#include <json/json.h>
#include "jsondata.h"

JsonRec::JsonRec() { setDefaultParams(); }
JsonRec::~JsonRec() { }

MocapRec *JsonRec::clone() {
  return static_cast<MocapRec*>(new JsonRec());
}

bool JsonRec::loadable(const char *recdir) {
  struct stat buffer;
  if(stat(STRING(recdir << "dataset.json"), &buffer))
    return false;
  if(stat(STRING(recdir << "../scene.json"), &buffer))
    return false;
  return true;
}

void JsonRec::load(const char *recdir) {
  dir = STRING(recdir);

  mid.load_json(STRING(dir << "../scene.json"));

  ifstream datafin;
  mlr::open(datafin, STRING(dir << "dataset.json"));

  Json::Reader reader;
  Json::Value root;
  CHECK(reader.parse(datafin, root), "Failed to parse configuration: " << reader.getFormattedErrorMessages());

  const StringA &sensors = mid.sensors();
  nsensors = sensors.N;
  nframes = root["data"].size();
  uint thinning = *params.get<uint>("thinning");
  nframes_thin = nframes / thinning;


  //bool m;
  boolA pm(nsensors);
  pm.setZero(false);
  // uint currfnum;
  // int hsi;
  // double currtstamp;
  arr data, tstamp, observed;
  boolA missing;
  mlr::Array<intA> missingno(nsensors), missingf(nsensors);

  ors::Transformation T;
  data.resize(nframes, nsensors, 7);
  data.setZero();

  tstamp.resize(nframes);

  missing.resize(nframes, nsensors);
  missing.setZero(true);

  observed.resize(nframes, nsensors);
  observed.setZero(0);
  for(uint f = 0; f < nframes; f++) {
    const Json::Value &frame = root["data"][f];
    tstamp(f) = frame["time_sec"].asDouble();

    for(const String &sensor: sensors) {
      uint sid = mid.sensor_id(sensor);

      missing(f, sid) = false;
      observed(f, sid) = 1;

      // TODO this accounts for the difference in the world frame..
      // T.setZero();
      // T.addRelativeRotationDeg(90, 1, 0, 0);

      data[f][sid]() = {
        frame["poses"][sensor]["pose"][0][0].asDouble(),
        frame["poses"][sensor]["pose"][0][1].asDouble(),
        frame["poses"][sensor]["pose"][0][2].asDouble(),
        frame["poses"][sensor]["pose"][1][3].asDouble(),
        frame["poses"][sensor]["pose"][1][0].asDouble(),
        frame["poses"][sensor]["pose"][1][1].asDouble(),
        frame["poses"][sensor]["pose"][1][2].asDouble()
      };
      // data[f][sid]() = ARR(
      //   frame["poses"][sensor]["pose"][0][0].asDouble(),
      //   frame["poses"][sensor]["pose"][0][1].asDouble(),
      //   frame["poses"][sensor]["pose"][0][2].asDouble(),
      //   frame["poses"][sensor]["pose"][1][3].asDouble(),
      //   frame["poses"][sensor]["pose"][1][0].asDouble(),
      //   frame["poses"][sensor]["pose"][1][1].asDouble(),
      //   frame["poses"][sensor]["pose"][1][2].asDouble()
      // );
    }

  //   // TODO append stuff, and then set it according to the indices in ID.
  //   for(uint sid = 0; sid < nsensors; sid++) {
  //     const String &sensor = mid.sensors().elem(sid);
  //
  //     m = !frame.isMember(sensor);
  //
  //     if(m && pm(is)) {
  //       missingno(is).last()++;
  //     }
  //     else if(m && !pm(is)) {
  //       missingno(is).append(1);
  //       missingf(is).append(f);
  //     }
  //   }
  }

  // setting quaternions as a continuous path on the 4d sphere
  arr dataquat, dataquatprev;
  dataquatprev = data[0].sub(0, -1, 3, -1);
  for(uint f = 1; f < data.d0; f++) {
    for(uint sid = 0; sid < data.d1; sid++) {
      dataquat.referToSub(data.subDim(f, sid)(), 3, -1);
      if(sum(dataquat % dataquatprev[sid]) < 0)
        dataquat *= -1.;
      // if(!length(dataquatprev[sid]) || length(dataquat))
      // TODO check!!! I think this is also ok
      if(length(dataquat))
        dataquatprev[sid]() = dataquat;
    }
  }

#if 0
  String interpolate = *params.get<String>("interpolate");
  if(interpolate == "true") { // interpolating missing observations
    NIY;
    // for(uint i = 0; i < nsensors; i++) {
    //   for(uint j = 0; j < missingno(i).N; j++) {
    //     uint t = missingf(i).elem(j);
    //     uint no = missingno(i).elem(j);
    //     if(t == 0) // set all equal to first
    //       for(uint tt = 0; tt < no; tt++)
    //         data[tt][i] = data[no][i];
    //     else if(t+no < nframes) { // interpolate between t-1 and t+missingno(i)
    //       arr s0 = data[t-1][i];
    //       arr sF = data[t+no][i];
    //       ors::Quaternion q0(s0(3), s0(4), s0(5), s0(6));
    //       ors::Quaternion qF(sF(3), sF(4), sF(5), sF(6));
    //       ors::Quaternion qt;

    //       arr diff = sF - s0;
    //       for(uint tt = 0; tt < no; tt++) {
    //         data[t+tt][i] = s0 + diff*(tt+1.)/(no+1.);
    //         qt.setInterpolate((tt+1.)/(no+1.), q0, qF);
    //         data(t+tt, i, 3) = qt.w;
    //         data(t+tt, i, 4) = qt.x;
    //         data(t+tt, i, 5) = qt.y;
    //         data(t+tt, i, 6) = qt.z;
    //       }
    //     }
    //     else // set all equal to last
    //       for(uint tt = 0; tt < no; tt++)
    //         data[nframes-tt-1][i] = data[nframes-no-1][i];
    //   }
    // }
  }
#endif

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

  // loading labeled targets
  if(root["targets"].isNull())
    cout << "No Targets!" << endl;
  else {
    kvgann.clear();

    Json::Value root = root["targets"];
    for(auto target_name: root.getMemberNames()) {
      Json::Value root = root[target_name];
      String target;
      target << target_name;

      targets.append(target);

      StringA &a_targets = *(new StringA());
      StringA &o_targets = *(new StringA());

      for(auto agent_name: root.getMemberNames()) {
        Json::Value root = root[agent_name];
        String agent;
        agent << agent_name;

        a_targets.append(agent);
        for(auto object_name: root.getMemberNames()) {
          Json::Value root = root[object_name];
          String object;
          object << object_name;

          o_targets.append(object);

          Graph &targetkvg = *(new Graph());
          arr &ann = *(new arr(nframes));
          ann.setZero();

          cout << "root.size(): " << root.size() << endl;
          for(uint i = 0; root.size(); i++) {
            uint onat = root[i]["on@"].asUInt();
            uint offat = root[i]["off@"].asUInt();
            ann.subRef(onat, offat) = 1;
          }

          targetkvg.append("ann", &ann);
          // kvgann.append(STRINGS(target, agent, object), &targetkvg);
          kvgann.append({target, agent, object}, {}, &targetkvg, false);
        }
      }

      agent_targets.append((char*)target, &a_targets);
      object_targets.append((char*)target, &o_targets);

      // ann = new arr(nframes);
      // ann->setZero();
      // if(!agent_targets.getNode(target))
      //   agent_targets.append(target, new StringA());
      // if(!object_targets.getNode(target))
      //   object_targets.append(target, new StringA());
      // StringA &a_targets = *agent_targets.getValue<StringA>(target);
      // StringA &o_targets = *object_targets.getValue<StringA>(target);

      // if(!a_targets.contains(pair->keys(1)))
      //   a_targets.append(pair->keys(1));
      // if(!o_targets.contains(pair->keys(2)))
      //   o_targets.append(pair->keys(2));
      // for(Node *lock: *pair->getValue<Graph>()) {
      //   from = (uint)*lock->getValue<Graph>()->getValue<double>("from");
      //   to = (uint)*lock->getValue<Graph>()->getValue<double>("to");
      //   ann->subRef(from, to) = 1;
      // }
      // pair->getValue<Graph>()->append("ann", ann);
    }
  }
  // loading annotation, if any..
  // TODO use the json file!!!!
  // try {
    // FILE(STRING(recdir << "ann_kvg.kvg")) >> kvgann;
    // uint from, to;

    // arr *ann;
    // for(Node *pair: kvgann) {
    //   ann = new arr(nframes);
    //   ann->setZero();
    //   if(!agent_targets.getNode(pair->keys(0)))
    //     agent_targets.append(pair->keys(0), new StringA());
    //   if(!object_targets.getNode(pair->keys(0)))
    //     object_targets.append(pair->keys(0), new StringA());
    //   StringA &a_targets = *agent_targets.getValue<StringA>(pair->keys(0));
    //   StringA &o_targets = *object_targets.getValue<StringA>(pair->keys(0));
    //   if(!a_targets.contains(pair->keys(1)))
    //     a_targets.append(pair->keys(1));
    //   if(!o_targets.contains(pair->keys(2)))
    //     o_targets.append(pair->keys(2));
    //   for(Node *lock: *pair->getValue<Graph>()) {
    //     from = (uint)*lock->getValue<Graph>()->getValue<double>("from");
    //     to = (uint)*lock->getValue<Graph>()->getValue<double>("to");
    //     ann->subRef(from, to) = 1;
    //   }
    //   pair->getValue<Graph>()->append("ann", ann);
    // }
  // }
  // catch(const char *e) {
    // cout << "No annotations in " << recdir << "." << endl;
  // }
}

