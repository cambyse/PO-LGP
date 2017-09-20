#ifndef HRI_STATE_H
#define HRI_STATE_H

#include <Perception/percept.h>
#include <Roopi/roopi.h>
#include <Geo/geo.h>
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

struct HRIObject {
  // colors in lab
  static const cv::Mat3f labRed;
  static const cv::Mat3f labGreen;
  static const cv::Mat3f labBlue;
  static const cv::Mat3f labYellow;
  enum Color { nullcolor, red, green, blue, yellow };
  enum Size { nullsize, small, medium, large};
  mlr::Transformation inittransform;
    /*arr initsize;
  arr initcolor;*/
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


struct  OnTableState {
  double tableheight;
  std::vector<HRIObject> objects;
  OnTableState();
  unsigned int objN; // number of objects in the world
  HRIObject* getObj(std::string oid);
  HRIObject* getObj(HRIObject::Color c, HRIObject::Size s, int number=0);
  int countObjs(HRIObject::Color c, HRIObject::Size s);
  void initFromPercepts(PerceptL percepts);
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
