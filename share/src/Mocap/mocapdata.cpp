#include <Core/array.h>
#include <Core/util.h>
#include <Geo/geo.h>
#include <Core/graph.h>
#include <Core/registry.h>
#include <json/json.h>
#include <Ors/ors.h>
#include "mocapdata.h"

#define ONLY_OBSERVED

#define ENUM_KVG(elem) Target_kvg.append(#elem, new Target(elem));
ENUM_MACRO_CPP(Target)

// =============================================================================
// MocapID
//

// TODO better system? this should be G4-specific
#define HSI(hid, sid) (3 * (hid) + (sid))

struct MocapID::sMocapID {
  // TODO remove kvg in general?
  Graph kvg, kvg_hid, kvg_sid;
  StringA sensors,
          agents, agent_limbs, agent_digits,
          objects, sensors_struct, sensors_unstruct, object_parts;
  StringA sensor_type, sensor_mesh;
  StringA sensor_rn; // eventually removed
  mlr::Array<ors::Transformation> sensor_meshtr;
  boolA sensor_hastype, sensor_hasmesh, sensor_hasmeshtr;

  String root; // eventually removed

  uintA hsitoi, itohsi;

  void parseAgentSensor(const String &sensor, Graph &kvg);
  void parseObjectSensor(const String &sensor, Graph &kvg);

  void parseAgentSensor(const String &sensor, Json::Value properties);
  void parseObjectSensor(const String &sensor, Json::Value properties);

  // uint hsitoi(uint hsi);
  // uint itohsi(uint i);

  // int i(const char *sensor);
  // int hsi(const char *sensor);
  const char *sensor(uint hsi);
};

void MocapID::sMocapID::parseAgentSensor(const String &sensor, Graph &kvg) {
  CHECK_EQ(sensor(0), '/', "Sensor name must start with '/'.");
  StringA agent_hierarchy;

  char sensor_cstr[51];
  strcpy(sensor_cstr, sensor);
  char *pch = strtok(sensor_cstr, "/");
  agent_hierarchy.append(STRING(pch));
  while((pch = strtok(nullptr, "/")) != nullptr)
    agent_hierarchy.append(STRING(pch));

  CHECK_EQ(agent_hierarchy.N, 3, "Agent sensor must contain three hierarchy levels.");

  String sens;
  sens << "/" << agent_hierarchy(0);
  if(!agents.contains(sens))
    agents.append(sens);
  sens << "/" << agent_hierarchy(1);
  if(!agent_limbs.contains(sens))
    agent_limbs.append(sens);
  agent_digits.append(sensor);
  sensors.append(sensor);
  sensors_unstruct.append(sensor);

  // RowNumber (old hsi)
  uint rn = *kvg.getValue<double>("rn");
  uint old_rnn = sensor_rn.N;
  if(rn >= old_rnn) {
    sensor_rn.resizeCopy(rn+1);
    for(uint i = old_rnn+1; i < rn; i++)
      sensor_rn(i) = "none";
  }
  sensor_rn(rn) = sensor;

  // Type
  String type = *kvg.getValue<String>("type");
  sensor_type.append(type);
}

void MocapID::sMocapID::parseObjectSensor(const String &sensor, Graph &kvg) {
  CHECK_EQ(sensor(0), '/', "Sensor name must start with '/'.");
  StringA object_hierarchy;

  char sensor_cstr[51];
  strcpy(sensor_cstr, sensor);
  char *pch = strtok(sensor_cstr, "/");
  object_hierarchy.append(STRING(pch));
  while((pch = strtok(nullptr, "/")) != nullptr)
    object_hierarchy.append(STRING(pch));

  String sens;
  sens << "/" << object_hierarchy(0);
  if(!objects.contains(sens))
    objects.append(sens);
  object_parts.append(sensor);
  sensors.append(sensor);
  // TODO struct or unstruct according to property
  sensors_unstruct.append(sensor);

  // RowNumber (old hsi)
  uint rn = *kvg.getValue<double>("rn");
  uint old_rnn = sensor_rn.N;
  if(rn >= old_rnn) {
    sensor_rn.resizeCopy(rn+1);
    for(uint i = old_rnn+1; i < rn; i++)
      sensor_rn(i) = "none";
  }
  sensor_rn(rn) = sensor;

  // Type
  String type = *kvg.getValue<String>("type");
  sensor_type.append(type);

  // Mesh
  // bool root = (nullptr != kvg.getValue)
}

void MocapID::sMocapID::parseAgentSensor(const String &sensor, Json::Value properties) {
  CHECK_EQ(sensor(0), '/', "Sensor name must start with '/'.");
  StringA agent_hierarchy;

  char sensor_cstr[51];
  strcpy(sensor_cstr, sensor);
  char *pch = strtok(sensor_cstr, "/");
  agent_hierarchy.append(STRING(pch));
  while((pch = strtok(nullptr, "/")) != nullptr)
    agent_hierarchy.append(STRING(pch));

  CHECK_EQ(agent_hierarchy.N, 3, "Agent sensor must contain three hierarchy levels.");

  String sens;
  sens << "/" << agent_hierarchy(0);
  if(!agents.contains(sens))
    agents.append(sens);
  sens << "/" << agent_hierarchy(1);
  if(!agent_limbs.contains(sens))
    agent_limbs.append(sens);
  agent_digits.append(sensor);
  sensors.append(sensor);
  sensors_unstruct.append(sensor);

  // Type
  String type = STRING(properties["type"].asString());
  sensor_type.append(type);
  sensor_hastype.append(true);

  // Mesh
  sensor_mesh.append(String());
  sensor_hasmesh.append(false);

  // Mesh Transform
  ors::Transformation TI;
  TI.setZero();
  sensor_meshtr.append(TI);
  sensor_hasmeshtr.append(false);
}

void MocapID::sMocapID::parseObjectSensor(const String &sensor, Json::Value properties) {
  CHECK_EQ(sensor(0), '/', "Sensor name must start with '/'.");
  StringA object_hierarchy;

  char sensor_cstr[51];
  strcpy(sensor_cstr, sensor);
  char *pch = strtok(sensor_cstr, "/");
  object_hierarchy.append(STRING(pch));
  while((pch = strtok(nullptr, "/")) != nullptr)
    object_hierarchy.append(STRING(pch));

  String sens;
  sens << "/" << object_hierarchy(0);
  if(!objects.contains(sens))
    objects.append(sens);
  object_parts.append(sensor);
  sensors.append(sensor);
  // TODO struct or unstruct according to property
  sensors_unstruct.append(sensor);

  // Type
  String type = STRING(properties["type"].asString());
  sensor_type.append(type);
  sensor_hastype.append(true);

  // Mesh
  String mesh = STRING(properties["mesh"].asString());
  sensor_mesh.append(mesh);
  sensor_hasmesh.append(true);

  // Mesh transform
  ors::Transformation T, TI;
  T.setZero();
  T.pos.x = .001 * properties["transformation"][0][0].asDouble();
  T.pos.y = .001 * properties["transformation"][0][1].asDouble();
  T.pos.z = .001 * properties["transformation"][0][2].asDouble();
  T.rot.w = properties["transformation"][1][3].asDouble();
  T.rot.x = properties["transformation"][1][0].asDouble();
  T.rot.y = properties["transformation"][1][1].asDouble();
  T.rot.z = properties["transformation"][1][2].asDouble();
  // cout << "T: " << T << endl;
  TI.setInverse(T);
  // cout << "TI: " << TI << endl;
  sensor_meshtr.append(TI);
  sensor_hasmeshtr.append(true);
}

// uint MocapID::sMocapID::hsitoi(uint hsi) {
//   return hsitoi(hsi);
// }

// uint MocapID::sMocapID::itohsi(uint i) {
//   return itohsi(i);
// }

// int MocapID::sMocapID::i(const char *sensor) {
//   return hsitoi(hsi(sensor));
// }

// int MocapID::sMocapID::hsi(const char *sensor) {
//   uint *hid = kvg_hid.getValue<uint>(sensor);
//   uint *sid = kvg_sid.getValue<uint>(sensor);

//   return HSI(*hid, *sid);
// }

const char *MocapID::sMocapID::sensor(uint hsi) {
  return (const char *)sensors(hsitoi(hsi));
}

MocapID::MocapID(): s(new sMocapID()) { }
MocapID::~MocapID() { delete s; }

void MocapID::clear() {
  s->sensors.clear();

  s->agents.clear();
  s->agent_limbs.clear();
  s->agent_digits.clear();

  s->objects.clear();
  s->sensors_struct.clear();
  s->sensors_unstruct.clear();
  s->object_parts.clear();
}

// void readNode(Graph *i, uintA &hsitoi, uintA &itohsi, int ind) {
//   uint hid, sid, hsi, hsitoiN;
  
//   hid = (uint)*i->getValue<double>("hid");
//   sid = (uint)*i->getValue<double>("sid");
//   hsi = HSI(hid, sid);

//   if(hsi >= hsitoi.N) {
//     hsitoiN = hsitoi.N;
//     hsitoi.resizeCopy(hsi+1);
//     // hsitoi.subRef(hsitoiN, hsi)() = -1;
//   }
//   hsitoi(hsi) = ind;
//   itohsi.append(hsi);

//   // cout << "mapping hsi " << hsi << " i " << *i << endl;
// }

void MocapID::load(const char *meta) {
  String name_agent, name_limb, name_digit, name_object, name_part;
  Graph kvg;
  FILE(meta) >> kvg;

  // Loading new meta.kvg into new internal_representation
  Graph *kvg_agents = kvg.getValue<Graph>("agents");
  if(kvg_agents == nullptr)
    cout << "No Agents!" << endl;
  else
    for(Node* i: *kvg_agents)
      s->parseAgentSensor(i->keys(0), *i->getValue<Graph>());

  Graph *kvg_objects = kvg.getValue<Graph>("objects");
  if(kvg_objects == nullptr)
    cout << "No Objects!" << endl;
  else
    for(Node *i: *kvg_objects)
      s->parseObjectSensor(i->keys(0), *i->getValue<Graph>());
}

void MocapID::load_json(const char *json) {
  Json::Reader reader;
  Json::Value root;

  ifstream scenefin;
  mlr::open(scenefin, STRING(json));
  CHECK(reader.parse(scenefin, root), "Failed to parse configuration: " << reader.getFormattedErrorMessages());

  if(root["agents"].isNull())
    cout << "No Agents!" << endl;
  else
    for(auto key: root["agents"].getMemberNames())
      s->parseAgentSensor(STRING(key), root["agents"][key]);

  if(root["objects"].isNull())
    cout << "No Objects!" << endl;
  else
    for(auto key: root["objects"].getMemberNames())
      s->parseObjectSensor(STRING(key), root["objects"][key]);
  if(!root["root"].isNull())
    s->root = STRING(root["root"].asCString());
}

const String& MocapID::sensor(uint i) const { return s->sensors(i); }
const StringA& MocapID::sensors() const { return s->sensors; }
const StringA& MocapID::sensors_struct() const { return s->sensors_struct; }
const StringA& MocapID::sensors_unstruct() const { return s->sensors_unstruct; }

StringA MocapID::sensorsof(const char *base) const {
  StringA sensorsofbase;
  for(const String &sensor: sensors())
    if(sensor.startsWith(base))
      sensorsofbase.append(sensor);
  return sensorsofbase;
}

int MocapID::root_id() {
  cout << "root is " << s->root << endl;
  return sensor_id(s->root);
}

int MocapID::sensor_id(const char *sensor) const {
  return s->sensors.findValue(STRING(sensor));
}

int MocapID::sensor_rn(const char *sensor) const {
  // cout << s->sensor_rn << endl;
  return s->sensor_rn.findValue(STRING(sensor));
}

const String &MocapID::sensor_type(const char *sensor) const {
  int id = sensor_id(sensor);
  CHECK(id >= 0, STRING("Sensor '" << sensor << "' does not exist."));
  return s->sensor_type(id);
}

const String &MocapID::sensor_mesh(const char *sensor) const {
  int id = sensor_id(sensor);
  CHECK(id >= 0, STRING("Sensor '" << sensor << "' does not exist."));
  return s->sensor_mesh(id);
}

const ors::Transformation &MocapID::sensor_meshtr(const char *sensor) const {
  int id = sensor_id(sensor);
  CHECK(id >= 0, STRING("Sensor '" << sensor << "' does not exist."));
  if(s->sensor_hasmeshtr(id))
    return s->sensor_meshtr(id);
  return NoTransformation;
}

const String& MocapID::root() const {
  return s->root;
}

const StringA& MocapID::objects() const { return s->objects; }
const StringA& MocapID::object_parts() const { return s->object_parts; }

const StringA& MocapID::agents() const { return s->agents; }
const StringA& MocapID::agent_limbs() const { return s->agent_limbs; }
const StringA& MocapID::agent_digits() const { return s->agent_digits; }

void MocapID::write(std::ostream &os) const {
  os << " === ID === " << endl;
  os << " - sensors: " << sensors() << endl;
  os << " - sensors_struct: " << sensors_struct() << endl;
  os << " - sensors_unstruct: " << sensors_unstruct() << endl;
  os << " - agents: " << agents() << endl;
  os << " - agent_limbs: " << agent_limbs() << endl;
  os << " - agent_digits: " << agent_digits() << endl;
  os << " - objects: " << objects() << endl;
  os << " - object_parts: " << object_parts() << endl;
  os << " --- " << endl;
  os << " - sensor_type: " << s->sensor_type << endl;
  os << " - sensor_hasmesh: " << s->sensor_hasmesh << endl;
  os << " - sensor_mesh: " << s->sensor_mesh << endl;
  os << " - root: " << s->root << endl;
  // os << " - sensor_rn: " << s->sensor_rn << endl;
  os << " === END ===" << endl;
}

// =============================================================================
// MocapRec
//

MocapRec::MocapRec() { setDefaultParams(); }
MocapRec::~MocapRec() { }

void MocapRec::setDefaultParams() {
  params.clear();

  String interpolate = mlr::getParameter<String>("interpolate", STRING("true"));
  uint wlen = mlr::getParameter<double>("wlen", 120);
  uint thinning = mlr::getParameter<double>("thinning", 12);

  params.set("interpolate", interpolate);
  params.set("wlen", wlen);
  params.set("thinning", thinning);
}

// std::vector<std::string> inline split(const std::string &source, const char *delimiter = " ", bool keepEmpty = false) {
//   std::vector<std::string> results;

//   size_t prev = 0;
//   size_t next = 0;

//   while((next = source.find_first_of(delimiter, prev)) != std::string::npos) {
//     if(keepEmpty || (next - prev != 0))
//       results.push_back(source.substr(prev, next - prev));
//     prev = next + 1;
//   }

//   if(prev < source.size())
//     results.push_back(source.substr(prev));

//   return results;
// }

void MocapRec::save() {
  ofstream fout;
  arr pos, quat;
  Json::Value root, array, array_pos, array_quat, array_pose, value;
  // loop objects

  ors::KinematicWorld kw(STRING(dir << "world.ors"));

  for(const String &sensor: id().agent_digits()) {
    value.clear();
    value["type"] = (const char *)id().sensor_type(sensor);
    // value["mesh"] = (const char *)id().sensor_mesh(sensor);
    // value["meshscale"] = (const char *)id().sensor_mesh(sensor);
    root["agents"][sensor] = value;
  }

  for(const String &sensor: id().object_parts()) {
    // cout << "SENSOR NAME: " << kw.getShapeByName(STRING("sh"<<sensor))->name << endl;
    // cout << "SENSOR ATS: " << kw.getShapeByName(STRING("sh"<<sensor))->ats << endl;
    // cout << "SENSOR MESH: " << kw.getShapeByName(STRING("sh"<<sensor))->ats.getValue<mlr::FileToken>("mesh")->name << endl;

    String mesh = kw.getShapeByName(STRING("sh"<<sensor))->ats.getValue<mlr::FileToken>("mesh")->name;

    ors::Transformation TI;
    TI.setInverse(kw.getShapeByName(STRING("sh"<<sensor))->rel);

    array_pos.clear();
    array_pos.append(TI.pos.x);
    array_pos.append(TI.pos.y);
    array_pos.append(TI.pos.z);

    array_quat.clear();
    array_quat.append(TI.rot.x);
    array_quat.append(TI.rot.y);
    array_quat.append(TI.rot.z);
    array_quat.append(TI.rot.w);

    array_pose.clear();
    array_pose.append(array_pos);
    array_pose.append(array_quat);

    // cout << "SENSOR NAME: " << kw.getShapeByName(STRING("sh"<<sensor))->name << endl;
    // cout << "SENSOR TR: [ "
    //   << "[ " << kw.getShapeByName(STRING("sh"<<sensor))->rel.pos.x
    //   << ", " << kw.getShapeByName(STRING("sh"<<sensor))->rel.pos.y
    //   << ", " << kw.getShapeByName(STRING("sh"<<sensor))->rel.pos.z
    //   << " ], "
    //   << "[ " << kw.getShapeByName(STRING("sh"<<sensor))->rel.rot.w
    //   << ", " << kw.getShapeByName(STRING("sh"<<sensor))->rel.rot.x
    //   << ", " << kw.getShapeByName(STRING("sh"<<sensor))->rel.rot.y
    //   << ", " << kw.getShapeByName(STRING("sh"<<sensor))->rel.rot.z
    //   << " ] ]" << endl;

    value.clear();
    value["type"] = (const char *)id().sensor_type(sensor);
    // TODO add mesh name to the ID
    value["mesh"] = (const char *)mesh;
    // value["meshscale"] = (const char *)mesh;
    value["transformation"] = array_pose;
    root["objects"][sensor] = value;
  }
  if(id().root().N)
    root["root"] = (const char *)id().root();

  mlr::open(fout, STRING(dir << "../scene.json"));
  fout << root;
  fout.close();

  root.clear();
  arr &&tstamp = query("tstamp");

  // loop time indeces
  array.clear();
  // for(uint f = 0; f < 2; f++) {
  for(uint f = 0; f < nframes; f++) {
    value.clear();
    // loop objects

    // cout << "=== HERE ===" << endl;
    // cout << "id().sensors(): " << id().sensors() << endl;
    // cout << "======" << endl;

    value["time_sec"] = tstamp(f);
    for(const String &sensor: id().sensors()) {
      if(query("poseObs", sensor, f).scalar()) {
        pos = query("pos", sensor, f);
        quat = query("quat", sensor, f);

        array_pos.clear();
        array_pos.append(pos(0));
        array_pos.append(pos(1));
        array_pos.append(pos(2));

        array_quat.clear();
        array_quat.append(quat(1));
        array_quat.append(quat(2));
        array_quat.append(quat(3));
        array_quat.append(quat(0));

        array_pose.clear();
        array_pose.append(array_pos);
        array_pose.append(array_quat);

        value["poses"][sensor]["pose"] = array_pose;
        // TODO compute real velocity
        value["poses"][sensor]["velocity"] = 0;

        // TODO compute real relative velocity
        // TODO nope. don't do this. Grows as O(t * n^2)
        // for(const String &sensor2: id().sensors()) {
        //   if(sensor2 != sensor && query("poseObs", sensor2, f).scalar()) {
        //     // relative pose
        //     array_pos.clear();
        //     array_pos.append(0);
        //     array_pos.append(0);
        //     array_pos.append(0);

        //     array_quat.clear();
        //     array_quat.append(0);
        //     array_quat.append(0);
        //     array_quat.append(0);
        //     array_quat.append(0);

        //     array_pose.clear();
        //     array_pose.append(array_pos);
        //     array_pose.append(array_quat);

        //     value["poses_rel"][STRING(sensor << "_" << sensor2)]["relative_pose"] = array_pose;

        //     // relative velocity
        //     array_pos.clear();
        //     array_pos.append(0);
        //     array_pos.append(0);
        //     array_pos.append(0);

        //     value["poses_rel"][STRING(sensor << "_" << sensor2)]["relative_velocity_linear"] = array_pos;
        //     value["poses_rel"][STRING(sensor << "_" << sensor2)]["relative_velocity_angular"] = array_pos;
        //   }
        // }
      }
    }
    array.append(value);
  }
  root["data"] = array;

  // temporal segmentation target
  for(Node *pair: kvgann) {
    for(Node *lock: *pair->getValue<Graph>()) {
      if(lock->keys(0) == "lock") {
        value.clear();
        value["on@"] = (uint)*lock->getValue<Graph>()->getValue<double>("from");
        value["off@"] = (uint)*lock->getValue<Graph>()->getValue<double>("to");
        root["targets"][pair->keys(0)][pair->keys(1)][pair->keys(2)].append(value);
      }
    }
  }

  mlr::open(fout, STRING(dir << "dataset.json"));
  Json::FastWriter writer;
  // fout << root;
  fout << writer.write(root);
  fout.close();
}

const MocapID &MocapRec::id() const { return mid; }
MocapID &MocapRec::id() { return const_cast<MocapID &>(static_cast<const MocapRec&>(*this).id()); }
const MocapLabel &MocapRec::label() const { return mlabel; }
MocapLabel &MocapRec::label() { return const_cast<MocapLabel &>(static_cast<const MocapRec&>(*this).label()); }

// arr MocapRec::ann(Target type, const char *sensor1, const char *sensor2) const {
//   for(Node *pair: kvgann)
//     if(pair->keys(0) == Target_to_str(type)
//     && ((mid.sensorsof(pair->keys(2)).contains(STRING(sensor2))
//         && (mid.sensorsof(pair->keys(1)).contains(STRING(sensor1))
//           || pair->keys(1) == sensor1))
//       || (mid.sensorsof(pair->keys(1)).contains(STRING(sensor2))
//         && (mid.sensorsof(pair->keys(2)).contains(STRING(sensor1))
//           || pair->keys(2) == sensor1))))
//       return *pair->getValue<Graph>()->getValue<arr>("ann");
//   return arr();
// }

uint MocapRec::numSensors() const { return nsensors; }
uint MocapRec::numFrames(Thickness thickness) const {
  switch(thickness) {
    case THICK:
      return nframes;
      break;
    case THIN:
      return nframes_thin;
      break;
    default:
      HALT("No Thickness of this type");
  }
}
// uint MocapRec::numDim(const char *bam) { return kvg.getValue<arr>(STRINGS("bam", bam))->d2; }
uint MocapRec::numDim(const char *bam) { return kvg.getValue<arr>(StringA({"bam", bam}))->d2; }

void MocapRec::appendBam(const char *bam, const arr &data) {
  Node *i = kvg.getNode({"bam", bam});

  if(!i)
    kvg.append<arr>({"bam", bam}, {}, new arr(data), true);
  // else
  //   *i->getValue<arr>() = data; // replacing
  else
    for(uint is = 0; is < data.d0; is++)
      if(data[is].sparsity())
        (*i->getValue<arr>())[is]() = data[is]; // replacing
}

bool MocapRec::hasBam(const char *bam) {
  // return kvg.getNode(STRINGS("bam", bam)) != NULL;
  return kvg.getNode(StringA({"bam", bam})) != NULL;
}

arr MocapRec::query(const char *bam) {
  // Node *i = kvg.getNode(STRINGS("bam", bam));
  Node *i = kvg.getNode(StringA({"bam", bam}));
  CHECK(i != nullptr, STRING("BAM '" << bam << "' does not exist."));

  // if(0 == strcmp(bam, "pose")) {
  //   arr data, dataPos, dataQuat;

  //   dataPos.referTo(*kvg.getValue<arr>(STRINGS("bam", "pos")));
  //   dataQuat.referTo(*kvg.getValue<arr>(STRINGS("bam", "quat")));
  //   data.append(dataPos);
  //   data.append(dataQuat);
  //   data.reshape(nsensors, nframes, 7);
  //   return data;
  // }
  // return *kvg.getValue<arr>(STRINGS("bam", bam));
  return *kvg.getValue<arr>(StringA({"bam", bam}));
}

arr MocapRec::query(const char *type, const char *sensor) {
  // Node *i = kvg.getNode(STRINGS("bam", type));
  Node *i = kvg.getNode(StringA({"bam", type}));
  CHECK(i != nullptr, STRING("BAM '" << type << "' does not exist."));

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  // if(0 == strcmp(type, "pose")) {
  //   arr xPos, xQuat;
  //   xPos.referTo(*kvg.getValue<arr>("pos"));
  //   xQuat.referTo(*kvg.getValue<arr>("quat"));

  //   uint nframes = numFrames();

  //   arr x;
  //   /* x.append(xPos); */
  //   /* x.append(xQuat); */
  //   /* x.reshape(s->mid.sensors().N, s->nframes, 7); */
  //   /* return x[i]; */
  //   x.append(xPos[is]);
  //   x.append(xQuat[is]);
  //   x.reshape(nframes, 7);
  //   return x;
  // }
  return i->getValue<arr>()->operator[](is);
}

arr MocapRec::query(const char *type, const char *sensor, uint f) {
  // Node *i = kvg.getNode(STRINGS("bam", type));
  Node *i = kvg.getNode(StringA({"bam", type}));
  CHECK(i != nullptr, STRING("BAM '" << type << "' does not exist."));

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  // if(0 == strcmp(type, "pose")) {
  //   arr x;
  //   x.append(kvg.getValue<arr>("pos")->subDim(is, f));
  //   x.append(kvg.getValue<arr>("quat")->subDim(is, f));
  //   return x;
  // }
  arr &x = *i->getValue<arr>();
  if(x.nd == 2)
    // return ARR(x(is, f));
    return {x(is, f)};
  return x.subDim(is, f);
}

arr MocapRec::query(const char *type, const StringA &sensors) {
  arr x;
  for(String sensor: sensors)
    x.append(query(type, sensor));
  x.reshape(sensors.N, nframes, numDim(type));
  return x;
}

arr MocapRec::query(const char *type, const StringA &sensors, uint f) {
  arr x;
  for(String sensor: sensors)
    x.append(query(type, sensor, f));
  x.reshape(sensors.N, numDim(type));
  return x;
}

/* arr MocapRec::query(const char *type, const char *sensor1, const char *sensor2) { */
/*   // TODO check that the type works for 2 sensors.... */
/*   // e.g. check that it is not "poses" */

/*   Graph *skvg1 = s->kvg_sensors.getValue<Graph>(sensor1); */
/*   Graph *skvg2 = s->kvg_sensors.getValue<Graph>(sensor2); */
/*   CHECK(s->kvg.getNode(type) != NULL, STRING("BAM '" << type << "' does not exist.")); */
/*   CHECK(skvg1, STRING("Sensor '" << sensor1 << "' does not exist.")); */
/*   CHECK(skvg2, STRING("Sensor '" << sensor2 << "' does not exist.")); */

/*   uint hid1, sid1, i1; */
/*   uint hid2, sid2, i2; */
  
/*   hid1 = *skvg1->getValue<double>("hid"); */
/*   sid1 = *skvg1->getValue<double>("sid"); */
/*   i1 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid1, sid1)); */
  
/*   hid2 = *skvg2->getValue<double>("hid"); */
/*   sid2 = *skvg2->getValue<double>("sid"); */
/*   i2 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid2, sid2)); */

/*   return s->kvg.getValue<arr>(type)->subDim(i1, i2); */
/* } */

/* arr MocapData::query(const char *type, const char *sensor1, const char *sensor2, uint f) { */
/*   Graph *skvg1 = s->kvg_sensors.getValue<Graph>(sensor1); */
/*   Graph *skvg2 = s->kvg_sensors.getValue<Graph>(sensor2); */
/*   CHECK(s->kvg.getNode(type) != NULL, STRING("BAM '" << type << "' does not exist.")); */
/*   CHECK(skvg1, STRING("Sensor '" << sensor1 << "' does not exist.")); */
/*   CHECK(skvg2, STRING("Sensor '" << sensor2 << "' does not exist.")); */

/*   uint hid1, sid1, i1; */
/*   uint hid2, sid2, i2; */
  
/*   hid1 = *skvg1->getValue<double>("hid"); */
/*   sid1 = *skvg1->getValue<double>("sid"); */
/*   i1 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid1, sid1)); */
  
/*   hid2 = *skvg2->getValue<double>("hid"); */
/*   sid2 = *skvg2->getValue<double>("sid"); */
/*   i2 = s->kvg.getValue<arr>("hsitoi")->elem(HSI(hid2, sid2)); */

/*   return s->kvg.getValue<arr>("bam", type)->subDim(i1, i2, f); */
/* } */

void MocapRec::computeDPos(const char *frame_sensor, const char *sensor) {
  String typeDPos, typeDPosObs;
  typeDPos << frame_sensor << "_dPos";
  typeDPosObs << frame_sensor << "_dPosObs";

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  arr bamDPos(nsensors, nframes, numDim("pos"));
  arr bamDPosObs(nsensors, nframes);
  arr bamDPosis, bamDPosObsis;
  bamDPos.setZero();
  bamDPosObs.setZero();
  bamDPosis.referTo(bamDPos[is]());
  bamDPosObsis.referTo(bamDPosObs[is]());

#ifdef ONLY_OBSERVED
  arr posYX, quatX, obsX, obsY;
  posYX = query("pos", sensor) - query("pos", frame_sensor);
  quatX = query("quat", frame_sensor);
  obsX = query("poseObs", frame_sensor);
  obsY = query("posObs", sensor);
  ors::Vector pYX, dpos;
  ors::Quaternion qX;
  for(uint f = 0; f < nframes; f++) {
    if(obsX(f) && obsY(f)) {
      qX.set(quatX[f].p);
      pYX.set(posYX[f].p);
      dpos = qX.invert() * pYX;
      bamDPosis[f]() = {dpos.x, dpos.y, dpos.z};
      bamDPosObsis(f) = 1;
    }
  }
#else
  arr posYX, quatX;
  posYX = query("pos", sensor) - query("pos", frame_sensor);
  quatX = query("quat", frame_sensor);
  ors::Vector pYX, dpos;
  ors::Quaternion qX;
  for(uint f = 0; f < nframes; f++) {
    qX.set(quatX[f].p);
    pYX.set(posYX[f].p);
    dpos = qX.invert() * pYX;
    bamDPosis[f]() = {dpos.x, dpos.y, dpos.z};
  }
#endif

  appendBam(typeDPos, bamDPos);
  appendBam(typeDPosObs, bamDPosObs);
}

void MocapRec::computeDQuat(const char *frame_sensor, const char *sensor) {
  String typeDQuat, typeDQuatObs;
  typeDQuat << frame_sensor << "_dQuat";
  typeDQuatObs << frame_sensor << "_dQuatObs";

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, STRING("Sensor '" << sensor << "' does not exist."));

  arr bamDQuat(nsensors, nframes, numDim("quat"));
  arr bamDQuatObs(nsensors, nframes);
  arr bamDQuatis, bamDQuatObsis;
  bamDQuat.setZero();
  bamDQuatObs.setZero();
  bamDQuatis.referTo(bamDQuat[is]());
  bamDQuatObsis.referTo(bamDQuatObs[is]());

#ifdef ONLY_OBSERVED
  arr quatX, quatY, obsX, obsY;
  quatX = query("quat", frame_sensor);
  quatY = query("quat", sensor);
  obsX = query("quatObs", frame_sensor);
  obsY = query("quatObs", sensor);
  ors::Quaternion qX, qY, dquat;
  for(uint f = 0; f < nframes; f++) {
    if(obsX(f) && obsY(f)) {
      qX.set(quatX[f].p);
      qY.set(quatY[f].p);
      dquat = qX.invert() * qY;
      bamDQuatis[f]() = { dquat.w, dquat.x, dquat.y, dquat.z };
      bamDQuatObsis(f) = 1;
    }
  }
#else
  arr quatX, quatY;
  quatX = query("quat", frame_sensor);
  quatY = query("quat", sensor);
  ors::Quaternion qX, qY, dquat;
  for(uint f = 0; f < nframes; f++) {
    qX.set(quatX[f].p);
    qY.set(quatY[f].p);
    dquat = qX.invert() * qY;
    bamDQuatis[f]() = { dquat.w, dquat.x, dquat.y, dquat.z };
  }
#endif

  appendBam(typeDQuat, bamDQuat);
  appendBam(typeDQuatObs, bamDQuatObs);
}

// void MocapRec::computeDDist() {
//   // TODO do this, shit...
//   String typeDdist, typeDdistObs;
//   typeDDist << frame_sensor << "_dDist";
//   typeDDistObs << frame_sensor << "_dDistObs";

//   int is = mid.sensor_id(sensor);
//   CHECK(is >= 0, STRING("Sensor '" << sensor << "' does not exist."));

//   arr bamDQuat(nsensors, nframes, numDim("quat"));
//   arr bamDQuatObs(nsensors, nframes);
//   arr bamDQuatis, bamDQuatObsis;
//   bamDQuat.setZero();
//   bamDQuatObs.setZero();
//   bamDQuatis.referTo(bamDQuat[is]());
//   bamDQuatObsis.referTo(bamDQuatObs[is]());


//   appendBam(typeDdist, bamDdist);
//   appendBam(typeIdist, bamIdist);
// }

void MocapRec::computeVarPast(const char *type, const char *sensor) {
  String typeObs = STRING(type << "Obs");
  String typeVarPast = STRING(type << "VarPast");
  String typeVarPastObs = STRING(typeVarPast << "Obs");
  
  arr bam, bamObs, bamVarPast, bamVarPastObs, window, windowMean, windowMeanRep;
  uint wlen, thinning, nframes, nframes_thin, ndim;
  
  wlen = *params.get<uint>("wlen");
  thinning = *params.get<uint>("thinning");

  nframes = numFrames(THICK);
  nframes_thin = numFrames(THIN);
  ndim = numDim(type);

  bam = query(type);
  bamObs = query(typeObs);
  bamVarPast.resize(bam.d0, nframes_thin);
  bamVarPast.setZero();
  bamVarPastObs.resize(bam.d0, nframes_thin);
  bamVarPastObs.setZero();

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, "Sensor '" << sensor << "' does not exist.");

  arr bamis, bamObsis, bamVarPastis, bamVarPastObsis;
  bamis.referTo(bam[is]());
  bamObsis.referTo(bamObs[is]());
  bamVarPastis.referTo(bamVarPast[is]());
  bamVarPastObsis.referTo(bamVarPastObs[is]());

  arr scalarProducts(nframes);

#ifdef ONLY_OBSERVED
  // TODO change this threshold
  uint obs = 0;
  uint TODO = 2;

  double S;
  arr M(ndim);
  M = S = 0;
  uint first_f_thin = 0;
  uint first_f = thinning-1;
  while(first_f < wlen-1) {
    first_f_thin++;
    first_f += thinning;
  }
  for(uint ff = first_f-wlen+1; ff <= first_f; ff++) {
    if(bamObsis(ff)) {
      scalarProducts(ff) = scalarProduct(bamis[ff], bamis[ff]);
      S += scalarProducts(ff);
      M += bamis[ff];
      obs++;
    }
  }
  if(obs >= TODO) {
    bamVarPastis(first_f_thin) = S / obs - scalarProduct(M, M) / (obs*obs);
    bamVarPastObsis(first_f_thin) = 1;
  }
  for(uint f_thin = first_f_thin+1; f_thin < nframes_thin; f_thin++) {
    uint f = (f_thin + 1) * thinning - 1;
    for(uint ff = f-wlen+1; ff <= f; ff++) {
      if(bamObsis(ff)) {
        scalarProducts(ff) = scalarProduct(bamis[ff], bamis[ff]);
        S += scalarProducts(ff);
        M += bamis[ff];
        obs++;
      }
      if(bamObsis(ff-thinning)) {
        S -= scalarProducts(ff-thinning);
        M -= bamis[ff-thinning];
        obs--;
      }
    }
    if(obs >= TODO) {
      bamVarPastis(f_thin) = S / obs - scalarProduct(M, M) / (obs*obs);
      bamVarPastObsis(f_thin) = 1;
    }
  }
#else
  uint N = wlen;
  uint NN = wlen*wlen;
  double S;
  arr M(ndim);
  M = S = 0;
  uint first_f_thin = 0;
  uint first_f = thinning-1;
  while(first_f < wlen-1) {
    first_f_thin++;
    first_f += thinning;
  }
  for(uint ff = first_f-wlen+1; ff <= first_f; ff++) {
    scalarProducts(ff) = scalarProduct(bamis[ff], bamis[ff]);
    S += scalarProducts(ff);
    M += bamis[ff];
  }
  bamVarPastis(first_f_thin) = S / N - scalarProduct(M, M) / NN;
  for(uint f_thin = first_f_thin+1; f_thin < nframes_thin; f_thin++) {
    uint f = (f_thin + 1) * thinning - 1;
    for(uint ff = f-wlen+1; ff <= f; ff++) {
      scalarProducts(ff) = scalarProduct(bamis[ff], bamis[ff]);
      S += scalarProducts(ff) - scalarProducts(ff-thinning);;
      M += bamis[ff] - bamis[ff-thinning];
    }
    bamVarPastis(f_thin) = S / N - scalarProduct(M, M) / NN;
  }
#endif

  appendBam(typeVarPast, bamVarPast);
  appendBam(typeVarPastObs, bamVarPastObs);
}

void MocapRec::computeVarPast(const StringA &types, const StringA &sensors) {
  for(const String &type: types)
    for(const String &sensor: sensors)
      computeVarPast(type, sensor);
}

void MocapRec::computeLinCoeffPast(const char *type, const char *sensor) {
  String typeObs = STRING(type << "Obs");
  String typeLinCoeffPast = STRING(type << "LinCoeffPast");
  String typeLinCoeffPastObs = STRING(typeLinCoeffPast << "Obs");
  
  arr bam, bamObs, bamLinCoeffPast, bamLinCoeffPastObs, window;
  uint ndim, wlen, thinning, /*nframes,*/ nframes_thin;
  
  wlen = *params.get<uint>("wlen");
  thinning = *params.get<uint>("thinning");

  nframes = numFrames(THICK);
  nframes_thin = numFrames(THIN);
  ndim = numDim(type);

  bam = query(type);
  bamObs = query(typeObs);
  bamLinCoeffPast.resize(bam.d0, nframes_thin, ndim);
  bamLinCoeffPast.setZero();
  bamLinCoeffPastObs.resize(bam.d0, nframes_thin);
  bamLinCoeffPastObs.setZero();

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, "Sensor '" << sensor << "' does not exist.");

  arr bamis, bamObsis, bamLinCoeffPastis, bamLinCoeffPastObsis;
  bamis.referTo(bam[is]());
  bamObsis.referTo(bamObs[is]());
  bamLinCoeffPastis.referTo(bamLinCoeffPast[is]());
  bamLinCoeffPastObsis.referTo(bamLinCoeffPastObs[is]());

#ifdef ONLY_OBSERVED
  // TODO change this threshold
  uint obs, TODO = 2;

  arr T(wlen, 2), TT, O(wlen, wlen), beta;
  for(uint t = 0; t < wlen; t++)
    T[t]() = {1., t+1.};
  TT = ~T;
  O.setZero();
  for(uint f_thin = 0; f_thin < nframes_thin; f_thin++) {
    uint f = (f_thin + 1) * thinning - 1;
    if(f < wlen-1) continue;
    obs = 0;
    for(uint t = 0; t < wlen; t++) {
      // O(t, t) = bamObsis(f-wlen+1+t);
      O(t, t) = bamObsis(f-wlen+1+t);
      obs += O(t, t);
    }
    if(obs >= TODO) {
      beta = inverse(TT * O * T) * TT * O * bamis.subRef(f-wlen+1, f);
      bamLinCoeffPastis[f_thin]() = beta[1];
      bamLinCoeffPastObsis(f_thin) = 1;
    }
  }
#else
  arr T(wlen, 2), TTTITT, beta;
  for(uint t = 0; t < wlen; t++)
    T[t]() = {1., t+1.};
    // T[t]() = ARR(1, t+1);
  TTTITT = inverse(~T * T) * ~T;
  for(uint f_thin = 0; f_thin < nframes_thin; f_thin++) {
    uint f = (f_thin + 1) * thinning - 1;
    if(f < wlen-1) continue;
    beta = TTTITT * bamis.subRef(f-wlen+1, f);
    bamLinCoeffPastis[f_thin]() = beta[1];
  }
#endif

  appendBam(typeLinCoeffPast, bamLinCoeffPast);
  appendBam(typeLinCoeffPastObs, bamLinCoeffPastObs);
}

void MocapRec::computeLinCoeffPast(const StringA &types, const StringA &sensors) {
  for(const String &type: types)
    for(const String &sensor: sensors)
      computeLinCoeffPast(type, sensor);
}

void MocapRec::computeVarFuture(const char *type, const char *sensor) {
  String typeObs = STRING(type << "Obs");
  String typeVarFuture = STRING(type << "VarFuture");
  String typeVarFutureObs = STRING(typeVarFuture << "Obs");
  
  arr bam, bamObs, bamVarFuture, bamVarFutureObs, window, windowMean, windowMeanRep;
  uint thinning, nframes, nframes_thin, ndim;
  
  thinning = *params.get<uint>("thinning");

  nframes = numFrames(THICK);
  nframes_thin = numFrames(THIN);
  ndim = numDim(type);

  bam = query(type);
  bamObs = query(typeObs);
  bamVarFuture.resize(bam.d0, nframes_thin);
  bamVarFuture.setZero();
  bamVarFutureObs.resize(bam.d0, nframes_thin);
  bamVarFutureObs.setZero();

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, "Sensor '" << sensor << "' does not exist.");

  arr bamis, bamObsis, bamVarFutureis, bamVarFutureObsis;
  bamis.referTo(bam[is]());
  bamObsis.referTo(bamObs[is]());
  bamVarFutureis.referTo(bamVarFuture[is]());
  bamVarFutureObsis.referTo(bamVarFutureObs[is]());

#ifdef ONLY_OBSERVED
  uint obs = 0;
  uint TODO = 2;

  double S;
  arr M(ndim);
  M = S = 0;
  for(uint f_thin = nframes_thin-1; f_thin != -1u; f_thin--) {
    uint f = (f_thin + 1) * thinning - 1;
    for(uint ff = f; ff < mlr::MIN(f+thinning, nframes); ff++) {
      // if(bamObsis(nframes-ff)) {
      if(bamObsis(ff)) {
        S += scalarProduct(bamis[ff], bamis[ff]);
        M += bamis[ff];
        // S += scalarProduct(bamis[nframes-ff], bamis[nframes-ff]);
        // M += bamis[nframes-ff];
        obs++;
      }
    }
    if(obs >= TODO) {
      bamVarFutureis(f_thin) = S / obs - scalarProduct(M, M) / (obs*obs);
      bamVarFutureObsis(f_thin) = 1;
    }
  }
#else
  uint N;
  double S;
  arr M(ndim);
  M = S = 0;
  for(uint f_thin = nframes_thin-1; f_thin != -1u; f_thin--) {
    uint f = (f_thin + 1) * thinning - 1;
    N = nframes - f;
    for(uint ff = f; ff < mlr::MIN(f+thinning, nframes); ff++) {
      S += scalarProduct(bamis[ff], bamis[ff]);
      M += bamis[ff];
    }
    bamVarFutureis(f_thin) = S / N - scalarProduct(M, M) / (N*N);
  }
#endif

  appendBam(typeVarFuture, bamVarFuture);
  appendBam(typeVarFutureObs, bamVarFutureObs);
}

void MocapRec::computeVarFuture(const StringA &types, const StringA &sensors) {
  for(const String &type: types)
    for(const String &sensor: sensors)
      computeVarFuture(type, sensor);
}

void MocapRec::computeLinCoeffFuture(const char *type, const char *sensor) {
  String typeObs = STRING(type << "Obs");
  String typeLinCoeffFuture = STRING(type << "LinCoeffFuture");
  String typeLinCoeffFutureObs = STRING(typeLinCoeffFuture << "Obs");
  
  arr bam, bamObs, bamLinCoeffFuture, bamLinCoeffFutureObs, window;
  uint ndim, thinning, nframes, nframes_thin;
  
  thinning = *params.get<uint>("thinning");

  nframes = numFrames(THICK);
  nframes_thin = numFrames(THIN);
  ndim = numDim(type);

  bam = query(type);
  bamObs = query(typeObs);
  bamLinCoeffFuture.resize(bam.d0, nframes_thin, ndim);
  bamLinCoeffFuture.setZero();
  bamLinCoeffFutureObs.resize(bam.d0, nframes_thin);
  bamLinCoeffFutureObs.setZero();

  int is = mid.sensor_id(sensor);
  CHECK(is >= 0, "Sensor '" << sensor << "' does not exist.");

  arr bamis, bamObsis, bamLinCoeffFutureis, bamLinCoeffFutureObsis;
  bamis.referTo(bam[is]());
  bamObsis.referTo(bamObs[is]());
  bamLinCoeffFutureis.referTo(bamLinCoeffFuture[is]());
  bamLinCoeffFutureObsis.referTo(bamLinCoeffFutureObs[is]());

#ifdef ONLY_OBSERVED
  uint obs = 0;
  uint TODO = 2;

  // TODO FIX THIS STILL!!!!
  arr TTT(2, 2), TTw(2, ndim), beta, phi_ff;
  TTT = 1;
  TTw[0]() = TTw[1]() = bamis[nframes-1];
  for(uint f_thin = nframes-1; f_thin != -1u; f_thin--) {
    uint f = (f_thin + 1) * thinning - 1;
    if(f >= nframes - 1)
      continue;
    for(uint ff = mlr::MIN(f+thinning, nframes-2); ff != f-1u; ff--) {
      if(bamObsis(ff)) {
      // if(bamObsis(nframes-ff)) {
        // phi_ff = ARR(1, ff);
        // phi_ff = ARR(1, nframes-ff);
        TTT += (phi_ff ^ phi_ff);
        TTw += (phi_ff ^ bamis[ff]);
        // TTw += (phi_ff ^ bamis[nframes-ff]);
        obs++;
      }
    }
    if(obs >= TODO) {
      beta = inverse(TTT) * TTw;
      bamLinCoeffFutureis[f_thin]() = beta[1];
      bamLinCoeffFutureObsis(f_thin) = 1;
    }
  }
#else
  arr TTT(2, 2), TTw(2, ndim), beta, phi_ff;
  TTT = 1;
  TTw[0]() = TTw[1]() = bamis[nframes-1];
  for(uint f_thin = nframes-1; f_thin != -1u; f_thin--) {
    uint f = (f_thin + 1) * thinning - 1;
    if(f >= nframes - 1)
      continue;
    for(uint ff = mlr::MIN(f+thinning, nframes-2); ff != f-1u; ff--) {
      phi_ff = {1., (double)(nframes-ff)};
      TTT += (phi_ff ^ phi_ff);
      TTw += (phi_ff ^ bamis[ff]);
    }
    beta = inverse(TTT) * TTw;
    bamLinCoeffFutureis[f_thin]() = beta[1];
  }
#endif
  // cout << bamLinCoeffFutureis << endl;
  // exit(1);

  appendBam(typeLinCoeffFuture, bamLinCoeffFuture);
  appendBam(typeLinCoeffFutureObs, bamLinCoeffFutureObs);
}

void MocapRec::computeLinCoeffFuture(const StringA &types, const StringA &sensors) {
  for(const String &type: types)
    for(const String &sensor: sensors)
      computeLinCoeffFuture(type, sensor);
}

MocapSeq *MocapRec::seq(const char *sens1, const char *sens2) {
  MocapSeq *seq = new MocapSeq(*this);
  seq->init(sens1, sens2);
  return seq;
}

MocapSeqL MocapRec::seqlist(const char *obj1, const char *obj2) {
  MocapSeqL seqlist;

  bool checklist[nsensors][nsensors];
  uint i1, i2;
  for(i1 = 0; i1 < nsensors; i1++)
    for(i2 = 0; i2 < nsensors; i2++)
      checklist[i1][i2] = false;

  StringA sensors1 = (obj1? mid.sensorsof(obj1): mid.sensors());
  StringA sensors2 = (obj2? mid.sensorsof(obj2): mid.sensors());
  for(const String &sens1: sensors1) {
    for(const String &sens2: sensors2) {
      i1 = mid.sensor_id(sens1);
      i2 = mid.sensor_id(sens2);
      if(i1 != i2 && !checklist[i1][i2]) {
        checklist[i1][i2] = checklist[i2][i1] = true;
        seqlist.append(seq(sens1, sens2));
      }
    }
  }
  return seqlist;
}

ors::KinematicWorld *MocapRec::newKW() {
  ors::KinematicWorld *kw = new ors::KinematicWorld(STRING(dir << "world.ors"));

  // Overwrite with mesh transformations found in scene.json
  // for(const String &sensor: id().sensors()) {
  //   const ors::Transformation &T = id().sensor_meshtr(sensor);
    // if(&T) {
    //   ors::Shape *sh = kw->getShapeByName(STRING("sh"<<sensor));
      // TODO fix this..
      // cout << "--" << endl;
      // cout << "sh->rel: " << sh->rel << endl;
      // cout << "T: " << T << endl;
      // sh->rel = T;
    // }
  // }

  return kw;
}

void MocapRec::write(std::ostream &os) const {
  os << "MocapRec: " << dir << endl;
  os << " - num sensors:     " << numSensors() << endl;
  os << " - num frames:      " << numFrames(THICK) << endl;
  os << " - num frames_thin: " << numFrames(THIN) << endl;
  os << " - id():" << endl;
  os << id() << endl;
}

// =============================================================================
// MocapData
//

MocapData::MocapData() { }
MocapData::~MocapData() { }

StringA &MocapData::base() { return bases; }

MocapRec &MocapData::rec(const char *recdir) {
  Node *i = kvg.getNode(recdir);
  if(i) return *i->getValue<MocapRec>();

  for(const String &base: bases) {
    String dir(STRING(base << recdir << "/"));
    for(MocapRec *recp: reclist) {
      if(recp->loadable(dir)) {
        cout << "found at: " << dir << endl;
        MocapRec *mrec = recp->clone();
        mrec->load(dir);
        kvg.append((char*)dir, mrec);
        return *mrec;
      }
    }
  }
  HALT("Recording class not defined for '" << recdir << "'");

  // String dir(STRING(basedir << recdir << "/"));
  // for(MocapRec *recp: reclist) {
  //   if(recp->loadable(dir)) {
  //     MocapRec &mrec = *recp->clone();
  //     mrec.load(STRING(basedir << recdir << "/"));
  //     kvg.append(recdir, &mrec);
  //     return mrec;
  //   }
  // }
  // HALT("Recording class not defined for '" << dir << "'");
}

void MocapData::write(std::ostream &os) const {
  os << "MocapData" << endl;
  for(Node *i: kvg)
    os << " * " << i->keys << endl;
}

// =============================================================================
// MocapSeq
//

MocapSeq::MocapSeq(MocapRec &_rec): rec(_rec) { }
MocapSeq::~MocapSeq() { }

void MocapSeq::init(const char *sens1, const char *sens2) {
  sensor1 = STRING(sens1);
  sensor2 = STRING(sens2);

  // rawdata = rec.query("pose", STRINGS(sens1, sens2))
  rawdata = rec.query("pose", StringA({sens1, sens2}));
  clearFeatData();

  uint thinning = *rec.params.get<uint>("thinning");
  params.set("thinning", thinning);

  nframes = rec.numFrames(THICK);
  nframes_thin = rec.numFrames(THIN);
}

void MocapSeq::setAnn(const String &target) {
  // arr &ann = *rec.label().getValue<arr>(STRINGS(target, sensor1, sensor2));
  arr &ann = *rec.label().getValue<arr>(StringA({target, sensor1, sensor2}));
  if(!ann.N)
    ann_thin.resize(0);
  else {
    ann_thin.resize(nframes_thin);
    uint thinning = *params.get<uint>("thinning");
    for(uint f_thin = 0; f_thin < nframes_thin; f_thin++)
      ann_thin(f_thin) = ann((f_thin+1) * thinning - 1);
  }
}

void MocapSeq::clearFeatData() {
  featdata.clear();
  data.clear();
};

void MocapSeq::appendFeatData(const arr &newFeatData) {
  if(!featdata.N) {
    featdata = newFeatData;
  }
  else {
    arr oldFeatData = featdata;
    featdata.clear();
    for(uint f_thin = 0; f_thin < nframes_thin; f_thin++) {
      featdata.append(oldFeatData[f_thin]);
      featdata.append(newFeatData[f_thin]);
    }
    featdata.reshape(nframes_thin, featdata.N / nframes_thin);
  }
  data = featdata;
}

void MocapSeq::appendObs() {
  arr obs1 = rec.query("poseObs", sensor1);
  arr obs2 = rec.query("poseObs", sensor2);
  // arr newFeatData(nframes_thin, 1);
  // for(uint f_thin = 0; f_thin < nframes_thin; f_thin++)
  //   newFeatData(f_thin, 0) = (obs1(f_thin) && obs2(f_thin));
  arr newFeatData = obs1 % obs2;
  newFeatData.reshape(nframes_thin, 1);
  appendFeatData(newFeatData);
}

// void MocapSeq::appendDDist() {
//   rec.computeDDist();

//   String sensor1_dDist;
//   sensor1_dDist << sensor1 << "_dDist";

//   // arr ddist = rec.query(sensor1_ddist, sensor2);
//   // arr newFeatData(f_thin, 1);
//   // for(uint f_thin = 0; f_thin < nframes_thin; f_thin++)
//   //   newFeatData(f_thin, 0) = ddist(f_thin);
//   arr newFeatData = rec.query(sensor_dDist, sensor2);
//   newFeatData.reshape(nframes_thin, 1);
//   appendFeatData(newFeatData);
// }

// void MocapSeq::appendIDist() {
//   rec.computeIDist();

//   String sensor1_iDist;
//   sensor1_iDist << sensor1 << "_iDist";

//   // arr idist = rec.query(sensor1_idist, sensor2);
//   // arr newFeatData(f_thin, 1);
//   // for(uint f_thin = 0; f_thin < nframes_thin; f_thin++)
//   //   newFeatData(f_thin, 0) = idist(f_thin);
//   arr newFeatData = rec.query(sensor1_iDist, sensor2);
//   newFeatData.reshape(nframes_thin, 1);
//   appendFeatData(newFeatData);
// }

void MocapSeq::appendVarPast() {
  rec.computeVarPast(StringA({"pos", "quat"}), StringA({sensor1, sensor2}));

  String sensor1_dPos, sensor1_dQuat, sensor1_dPosVarPast, sensor1_dQuatVarPast, sensor1_dPosVarPastObs;
  sensor1_dPos << sensor1 << "_dPos";
  sensor1_dQuat << sensor1 << "_dQuat";
  sensor1_dPosVarPast << sensor1_dPos << "VarPast";
  sensor1_dQuatVarPast << sensor1_dQuat << "VarPast";
  sensor1_dPosVarPastObs << sensor1_dPosVarPast << "Obs";

  rec.computeDPos(sensor1, sensor2);
  rec.computeDQuat(sensor1, sensor2);
  rec.computeVarPast(sensor1_dPos, sensor2);
  rec.computeVarPast(sensor1_dQuat, sensor2);

  arr sensor1_posVarPast = rec.query("posVarPast", sensor1);
  arr sensor1_quatVarPast = rec.query("quatVarPast", sensor1);
  arr sensor2_posVarPast = rec.query("posVarPast", sensor2);
  arr sensor2_quatVarPast = rec.query("quatVarPast", sensor2);
  arr delta_posVarPast = rec.query(sensor1_dPosVarPast, sensor2);
  arr delta_quatVarPast = rec.query(sensor1_dQuatVarPast, sensor2);

  arr sensor1_posVarPastObs = rec.query("posVarPastObs", sensor1);
  arr sensor2_posVarPastObs = rec.query("posVarPastObs", sensor2);
  arr delta_posVarPastObs = rec.query(sensor1_dPosVarPastObs, sensor2);

  arr newFeatData;
  for(uint f_thin = 0; f_thin < nframes_thin; f_thin++)
    newFeatData.append({
      sensor1_posVarPastObs(f_thin) + sensor2_posVarPastObs(f_thin),
      sensor1_posVarPast(f_thin) + sensor2_posVarPast(f_thin),
      sensor1_quatVarPast(f_thin) + sensor2_quatVarPast(f_thin),
      delta_posVarPastObs(f_thin),
      delta_posVarPast(f_thin),
      delta_quatVarPast(f_thin)
    });
  newFeatData.reshape(nframes_thin, newFeatData.N / nframes_thin);
  appendFeatData(newFeatData);
}

void MocapSeq::appendLinCoeffPast(bool sqr_feats) {
  // rec.computeLinCoeffPast(STRINGS("pos", "quat"), STRINGS(sensor1, sensor2));
  rec.computeLinCoeffPast(StringA({"pos", "quat"}), StringA({sensor1, sensor2}));

  String sensor1_dPos, sensor1_dQuat, sensor1_dPosLinCoeffPast, sensor1_dQuatLinCoeffPast, sensor1_dPosLinCoeffPastObs;
  sensor1_dPos << sensor1 << "_dPos";
  sensor1_dQuat << sensor1 << "_dQuat";
  sensor1_dPosLinCoeffPast << sensor1_dPos << "LinCoeffPast";
  sensor1_dQuatLinCoeffPast << sensor1_dQuat << "LinCoeffPast";
  sensor1_dPosLinCoeffPastObs << sensor1_dPosLinCoeffPast << "Obs";

  rec.computeDPos(sensor1, sensor2);
  rec.computeDQuat(sensor1, sensor2);
  rec.computeLinCoeffPast(sensor1_dPos, sensor2);
  rec.computeLinCoeffPast(sensor1_dQuat, sensor2);

  arr sensor1_posLinCoeffPast = rec.query("posLinCoeffPast", sensor1);
  arr sensor1_quatLinCoeffPast = rec.query("quatLinCoeffPast", sensor1);
  arr sensor2_posLinCoeffPast = rec.query("posLinCoeffPast", sensor2);
  arr sensor2_quatLinCoeffPast = rec.query("quatLinCoeffPast", sensor2);
  arr delta_posLinCoeffPast = rec.query(sensor1_dPosLinCoeffPast, sensor2);
  arr delta_quatLinCoeffPast = rec.query(sensor1_dQuatLinCoeffPast, sensor2);

  arr sensor1_posLinCoeffPastObs = rec.query("posLinCoeffPastObs", sensor1);
  arr sensor2_posLinCoeffPastObs = rec.query("posLinCoeffPastObs", sensor2);
  arr delta_posLinCoeffPastObs = rec.query(sensor1_dPosLinCoeffPastObs, sensor2);

  if(sqr_feats) {
    // sqr
    sensor1_posLinCoeffPast = sqr(sensor1_posLinCoeffPast);
    sensor1_quatLinCoeffPast = sqr(sensor1_quatLinCoeffPast);
    sensor2_posLinCoeffPast = sqr(sensor2_posLinCoeffPast);
    sensor2_quatLinCoeffPast = sqr(sensor2_quatLinCoeffPast);
    delta_posLinCoeffPast = sqr(delta_posLinCoeffPast);
    delta_quatLinCoeffPast = sqr(delta_quatLinCoeffPast);
  }
  else {
    // poor man's abs
    sensor1_posLinCoeffPast = elemWiseMax(sensor1_posLinCoeffPast, -sensor1_posLinCoeffPast);
    sensor1_quatLinCoeffPast = elemWiseMax(sensor1_quatLinCoeffPast, -sensor1_quatLinCoeffPast);
    sensor2_posLinCoeffPast = elemWiseMax(sensor2_posLinCoeffPast, -sensor2_posLinCoeffPast);
    sensor2_quatLinCoeffPast = elemWiseMax(sensor2_quatLinCoeffPast, -sensor2_quatLinCoeffPast);
    delta_posLinCoeffPast = elemWiseMax(delta_posLinCoeffPast, -delta_posLinCoeffPast);
    delta_quatLinCoeffPast = elemWiseMax(delta_quatLinCoeffPast, -delta_quatLinCoeffPast);
  }
  
  arr newFeatData;
  for(uint f_thin = 0; f_thin < nframes_thin; f_thin++) {
    newFeatData.append(sensor1_posLinCoeffPastObs(f_thin) + sensor2_posLinCoeffPastObs(f_thin));
    newFeatData.append(sensor1_posLinCoeffPast[f_thin] + sensor2_posLinCoeffPast[f_thin]);
    newFeatData.append(sensor1_quatLinCoeffPast[f_thin] + sensor2_quatLinCoeffPast[f_thin]);
    newFeatData.append(delta_posLinCoeffPastObs(f_thin));
    newFeatData.append(delta_posLinCoeffPast[f_thin]);
    newFeatData.append(delta_quatLinCoeffPast[f_thin]);
  }
  newFeatData.reshape(nframes_thin, newFeatData.N/nframes_thin);
  appendFeatData(newFeatData);
}

void MocapSeq::appendVarFuture() {
  // rec.computeVarFuture(STRINGS("pos", "quat"), STRINGS(sensor1, sensor2));
  rec.computeVarFuture(StringA({"pos", "quat"}), StringA({sensor1, sensor2}));

  String sensor1_dPos, sensor1_dQuat, sensor1_dPosVarFuture, sensor1_dQuatVarFuture, sensor1_dPosVarFutureObs;
  sensor1_dPos << sensor1 << "_dPos";
  sensor1_dQuat << sensor1 << "_dQuat";
  sensor1_dPosVarFuture << sensor1_dPos << "VarFuture";
  sensor1_dQuatVarFuture << sensor1_dQuat << "VarFuture";
  sensor1_dPosVarFutureObs << sensor1_dPosVarFuture << "Obs";

  rec.computeDPos(sensor1, sensor2);
  rec.computeDQuat(sensor1, sensor2);
  rec.computeVarFuture(sensor1_dPos, sensor2);
  rec.computeVarFuture(sensor1_dQuat, sensor2);

  arr sensor1_posVarFuture = rec.query("posVarFuture", sensor1);
  arr sensor1_quatVarFuture = rec.query("quatVarFuture", sensor1);
  arr sensor2_posVarFuture = rec.query("posVarFuture", sensor2);
  arr sensor2_quatVarFuture = rec.query("quatVarFuture", sensor2);
  arr delta_posVarFuture = rec.query(sensor1_dPosVarFuture, sensor2);
  arr delta_quatVarFuture = rec.query(sensor1_dQuatVarFuture, sensor2);

  arr sensor1_posVarFutureObs = rec.query("posVarFutureObs", sensor1);
  arr sensor2_posVarFutureObs = rec.query("posVarFutureObs", sensor2);
  arr delta_posVarFutureObs = rec.query(sensor1_dPosVarFutureObs, sensor2);

  arr newFeatData;
  for(uint f_thin = 0; f_thin < nframes_thin; f_thin++)
    newFeatData.append({
      sensor1_posVarFutureObs(f_thin) + sensor2_posVarFutureObs(f_thin),
      sensor1_posVarFuture(f_thin) + sensor2_posVarFuture(f_thin),
      sensor1_quatVarFuture(f_thin) + sensor2_quatVarFuture(f_thin),
      delta_posVarFutureObs(f_thin),
      delta_posVarFuture(f_thin),
      delta_quatVarFuture(f_thin)
    });
  newFeatData.reshape(nframes_thin, newFeatData.N / nframes_thin);
  appendFeatData(newFeatData);
}

void MocapSeq::appendLinCoeffFuture(bool sqr_feats) {
  // rec.computeLinCoeffFuture(STRINGS("pos", "quat"), STRINGS(sensor1, sensor2));
  rec.computeLinCoeffFuture(StringA({"pos", "quat"}), StringA({sensor1, sensor2}));

  String sensor1_dPos, sensor1_dQuat, sensor1_dPosLinCoeffFuture, sensor1_dQuatLinCoeffFuture, sensor1_dPosLinCoeffFutureObs;
  sensor1_dPos << sensor1 << "_dPos";
  sensor1_dQuat << sensor1 << "_dQuat";
  sensor1_dPosLinCoeffFuture << sensor1_dPos << "LinCoeffFuture";
  sensor1_dQuatLinCoeffFuture << sensor1_dQuat << "LinCoeffFuture";
  sensor1_dPosLinCoeffFutureObs << sensor1_dPosLinCoeffFuture << "Obs";

  rec.computeDPos(sensor1, sensor2);
  rec.computeDQuat(sensor1, sensor2);
  rec.computeLinCoeffFuture(sensor1_dPos, sensor2);
  rec.computeLinCoeffFuture(sensor1_dQuat, sensor2);

  arr sensor1_posLinCoeffFuture = rec.query("posLinCoeffFuture", sensor1);
  arr sensor1_quatLinCoeffFuture = rec.query("quatLinCoeffFuture", sensor1);
  arr sensor2_posLinCoeffFuture = rec.query("posLinCoeffFuture", sensor2);
  arr sensor2_quatLinCoeffFuture = rec.query("quatLinCoeffFuture", sensor2);
  arr delta_posLinCoeffFuture = rec.query(sensor1_dPosLinCoeffFuture, sensor2);
  arr delta_quatLinCoeffFuture = rec.query(sensor1_dQuatLinCoeffFuture, sensor2);

  arr sensor1_posLinCoeffFutureObs = rec.query("posLinCoeffFutureObs", sensor1);
  arr sensor2_posLinCoeffFutureObs = rec.query("posLinCoeffFutureObs", sensor2);
  arr delta_posLinCoeffFutureObs = rec.query(sensor1_dPosLinCoeffFutureObs, sensor2);

  if(sqr_feats) {
    // sqr
    sensor1_posLinCoeffFuture = sqr(sensor1_posLinCoeffFuture);
    sensor1_quatLinCoeffFuture = sqr(sensor1_quatLinCoeffFuture);
    sensor2_posLinCoeffFuture = sqr(sensor2_posLinCoeffFuture);
    sensor2_quatLinCoeffFuture = sqr(sensor2_quatLinCoeffFuture);
    delta_posLinCoeffFuture = sqr(delta_posLinCoeffFuture);
    delta_quatLinCoeffFuture = sqr(delta_quatLinCoeffFuture);
  }
  else {
    // poor man's abs
    sensor1_posLinCoeffFuture = elemWiseMax(sensor1_posLinCoeffFuture, -sensor1_posLinCoeffFuture);
    sensor1_quatLinCoeffFuture = elemWiseMax(sensor1_quatLinCoeffFuture, -sensor1_quatLinCoeffFuture);
    sensor2_posLinCoeffFuture = elemWiseMax(sensor2_posLinCoeffFuture, -sensor2_posLinCoeffFuture);
    sensor2_quatLinCoeffFuture = elemWiseMax(sensor2_quatLinCoeffFuture, -sensor2_quatLinCoeffFuture);
    delta_posLinCoeffFuture = elemWiseMax(delta_posLinCoeffFuture, -delta_posLinCoeffFuture);
    delta_quatLinCoeffFuture = elemWiseMax(delta_quatLinCoeffFuture, -delta_quatLinCoeffFuture);
  }
  
  arr newFeatData;
  for(uint f_thin = 0; f_thin < nframes_thin; f_thin++) {
    newFeatData.append(sensor1_posLinCoeffFutureObs(f_thin) + sensor2_posLinCoeffFutureObs(f_thin));
    newFeatData.append(sensor1_posLinCoeffFuture[f_thin] + sensor2_posLinCoeffFuture[f_thin]);
    newFeatData.append(sensor1_quatLinCoeffFuture[f_thin] + sensor2_quatLinCoeffFuture[f_thin]);
    newFeatData.append(delta_posLinCoeffFutureObs(f_thin));
    newFeatData.append(delta_posLinCoeffFuture[f_thin]);
    newFeatData.append(delta_quatLinCoeffFuture[f_thin]);
  }
  newFeatData.reshape(nframes_thin, newFeatData.N/nframes_thin);
  appendFeatData(newFeatData);
}

void MocapSeq::write(std::ostream &os) const {
  os << "MocapSeq: " << endl;
  os << " * sensors: " << sensor1 << ", " << sensor2 << endl;
  os << " * dim: " << data.dim() << endl;
  os << " * ann: " << (ann.N? "yes": "no") << endl;
}

