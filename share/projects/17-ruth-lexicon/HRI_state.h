#ifndef HRI_STATE_H
#define HRI_STATE_H

#include <Perception/percept.h>
#include <Roopi/roopi.h>
#include <Geo/geo.h>
#include <string>
#include <vector>
#include <iostream>

// note that this is an old version of HRI_state.h used for the HAI 2017 language paper


struct HRIObject {
  enum Color { nullcolor, red, green, blue, yellow };
  enum Size { nullsize, small, medium, large};
  Color color;
  Size size;
  mlr::Vector pos;
  mlr::Quaternion rot;
  std::string id;
  HRIObject* below_obj=NULL;
  bool isBelow(HRIObject* s);
  void setByData(const arr& col, const arr& s, const mlr::Vector p, const mlr::Quaternion r);
  void setByName(Roopi&, const char* );
  friend std::ostream& operator<<(std::ostream& os, const HRIObject state);
};

class  OnTableState {
private:
  std::vector<HRIObject> objects;
public:
  HRIObject* getObj(std::string oid);
  HRIObject* getObj(HRIObject::Color c, HRIObject::Size s);
  void updateFromPercepts(PerceptL percepts);
  void updateFromModelworld(Roopi& R, const char* objname = NULL);
  void updateModelworldFromState();
  friend std::ostream& operator<<(std::ostream& os, const OnTableState state);
};

struct RobotState {
  HRIObject* left=NULL;
  HRIObject* right=NULL;
  bool containsObj(HRIObject* obj);
  void removeObj(HRIObject* obj);
};

#endif
