#include "g4id.h"
#include <Core/util.h>
#include <Core/keyValueGraph.h>

#define HSI(hid, sid) (3 * (hid) + (sid))

struct G4ID::sG4ID {
  KeyValueGraph kvg, kvg_sensors, kvg_sublimbs, kvg_suplimbs, kvg_digitsof, kvg_sensorsof;
  StringA sensors, struct_sensors, unstruct_sensors,
          subjects, objects,
          agents, limbs, digits;
  uintA hsitoi, itohsi;
};

G4ID::G4ID(): s(new sG4ID()) { }
G4ID::~G4ID() { delete s; }

void G4ID::clear() {
  s->kvg.clear();
  s->kvg_sensors.clear();
  s->kvg_sublimbs.clear();
  s->kvg_suplimbs.clear();
  s->kvg_digitsof.clear();
  s->kvg_sensorsof.clear();

  s->sensors.clear();
  s->struct_sensors.clear();
  s->unstruct_sensors.clear();

  s->subjects.clear();
  s->objects.clear();

  s->agents.clear();
  s->limbs.clear();
  s->digits.clear();
}

void readItem(KeyValueGraph *i, uintA &hsitoi, uintA &itohsi, int ind) {
  uint hid, sid, hsi, hsitoiN;
  
  hid = (uint)*i->getValue<double>("hid");
  sid = (uint)*i->getValue<double>("sid");
  hsi = HSI(hid, sid);

  if(hsi >= hsitoi.N) {
    hsitoiN = hsitoi.N;
    hsitoi.resizeCopy(hsi+1);
    hsitoi.subRange(hsitoiN, hsi)() = -1;
  }
  hsitoi(hsi) = ind;
  itohsi.append(hsi);
}

void G4ID::load(const char *meta) {
  String name_agent, name_limb, name_digit, name_object, name_part;
  MT::Array<KeyValueGraph*> kvg_agents, kvg_limbs, kvg_digits, kvg_objects, kvg_parts;
  uint i = 0;

  bool structured;

  FILE(meta) >> s->kvg;

  kvg_agents = s->kvg.getTypedValues<KeyValueGraph>("agent");
  for(KeyValueGraph *a: kvg_agents) {
    a->getValue(name_agent, "name");
    s->agents.append(name_agent);
    s->subjects.append(name_agent);

    s->kvg_suplimbs.append(name_agent, new String(""));
    s->kvg_sublimbs.append(name_agent, new StringA());

    s->kvg_digitsof.append(name_agent, new StringA());
    s->kvg_sensorsof.append(name_agent, new StringA());

    kvg_limbs = a->getTypedValues<KeyValueGraph>("limb");
    for(KeyValueGraph *l: kvg_limbs) {
      l->getValue(name_limb, "name");
      s->limbs.append(name_limb);
      s->subjects.append(name_limb);

      s->kvg_suplimbs.append(name_limb, new String(name_agent));
      s->kvg_sublimbs.append(name_limb, new StringA());
      s->kvg_sublimbs.getValue<StringA>(name_agent)->append(name_limb);

      s->kvg_digitsof.append(name_limb, new StringA());
      s->kvg_sensorsof.append(name_limb, new StringA());

      kvg_digits = l->getTypedValues<KeyValueGraph>("digit");
      for(KeyValueGraph *d: kvg_digits) {
        d->getValue(name_digit, "name");
        s->digits.append(name_digit);
        s->subjects.append(name_digit);
        s->sensors.append(name_digit);
        s->unstruct_sensors.append(name_digit);

        s->kvg_suplimbs.append(name_digit, new String(name_limb));
        s->kvg_sublimbs.append(name_digit, new StringA());
        s->kvg_sublimbs.getValue<StringA>(name_limb)->append(name_digit);

        s->kvg_digitsof.append(name_digit, new StringA());
        s->kvg_digitsof.getValue<StringA>(name_digit)->append(name_digit);
        s->kvg_digitsof.getValue<StringA>(name_limb)->append(name_digit);
        s->kvg_digitsof.getValue<StringA>(name_agent)->append(name_digit);

        s->kvg_sensorsof.append(name_digit, new StringA());
        s->kvg_sensorsof.getValue<StringA>(name_digit)->append(name_digit); 
        s->kvg_sensorsof.getValue<StringA>(name_limb)->append(name_digit); 
        s->kvg_sensorsof.getValue<StringA>(name_agent)->append(name_digit); 

        s->kvg_sensors.append(name_digit, d);

        readItem(d, s->hsitoi, s->itohsi, i++);
      }
    }
  }

  kvg_objects = s->kvg.getTypedValues<KeyValueGraph>("object");
  for(KeyValueGraph *o: kvg_objects) {
    o->getValue(name_object, "name");
    s->objects.append(name_object);
    if(o->getValue(structured, "structured") && structured) {
      kvg_parts = o->getTypedValues<KeyValueGraph>("part");
      s->kvg_sensorsof.append(name_object, new StringA());
      for(KeyValueGraph *p: kvg_parts) {
        p->getValue(name_part, "name");
        s->struct_sensors.append(name_part);
        s->sensors.append(name_part);

        s->kvg_sensorsof.getValue<StringA>(name_object)->append(name_part); 
        s->kvg_sensors.append(name_part, p);

        readItem(p, s->hsitoi, s->itohsi, i++);
      }
    }
    else {
      s->unstruct_sensors.append(name_object);
      s->sensors.append(name_object);

      s->kvg_sensors.append(name_object, o);
      s->kvg_sensorsof.append(name_object, new StringA());
      s->kvg_sensorsof.getValue<StringA>(name_object)->append(name_object); 

      readItem(o, s->hsitoi, s->itohsi, i++);
    }
  }
}

const StringA& G4ID::sensors() { return s->sensors; }
const StringA& G4ID::struct_sensors() { return s->struct_sensors; }
const StringA& G4ID::unstruct_sensors() { return s->unstruct_sensors; }
const StringA& G4ID::sensorsof(const String &obj) { return *s->kvg_sensorsof.getValue<StringA>(obj); }

const StringA& G4ID::subjects() { return s->subjects; }
const StringA& G4ID::objects() { return s->objects; }

const StringA& G4ID::agents() { return s->agents; }
const StringA& G4ID::limbs() { return s->limbs; }
const StringA& G4ID::digits() { return s->digits; }

const StringA& G4ID::digitsof(const String &limb) { return *s->kvg_digitsof.getValue<StringA>(limb); }
const StringA& G4ID::sublimbs(const String &limb) { return *s->kvg_sublimbs.getValue<StringA>(limb); }
const String& G4ID::suplimb(const String &limb) { return *s->kvg_suplimbs.getValue<String>(limb); }

uint G4ID::hsitoi(uint hsi) {
  return s->hsitoi(hsi);
}

uint G4ID::itohsi(uint i) {
  return s->itohsi(i);
}

int G4ID::i(const char *sensor) {
  return s->hsitoi(hsi(sensor));
}

int G4ID::hsi(const char *sensor) {
  KeyValueGraph *skvg = s->kvg_sensors.getValue<KeyValueGraph>(sensor);

  uint hid = *skvg->getValue<double>("hid");
  uint sid = *skvg->getValue<double>("sid");

  return HSI(hid, sid);
}

const char *G4ID::sensor(uint hsi) {
  return s->sensors(s->hsitoi(hsi));
}

arr G4ID::query(const arr &data, const String &sensor) {
  return data[i(sensor)];
}

arr G4ID::query(const arr &data, const StringA &sensors) {
  arr x;
  for(const String &sensor: sensors) {
    x.append(data[i(sensor)]);
  }
  uint nsensors = sensors.N;
  x.reshape(nsensors, x.N/nsensors);
  return x;
}

