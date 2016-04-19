#pragma once
#include <Core/array.h>
#include <Ors/ors.h>

#ifdef MLR_ROS_INDIGO
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif
#if MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#endif


struct FilterObject {
  enum FilterObjectType { cluster, plane, alvar };

  uint id;
  double relevance;
  ors::Transformation transform;
  FilterObjectType type;

  FilterObject(){}

//  FilterObject(Cluster cluster):FilterObject()  {
//    this->Cluster::mean = cluster.mean;
//    this->Cluster::frame_id = cluster.frame_id;
//    this->Cluster::points = cluster.points;
//    this->type = FilterObjectType::cluster;
//  }

//  FilterObject(Alvar alvar):FilterObject()
//  {
//    this->Alvar::position = alvar.position;
//    this->Alvar::quaternion = alvar.quaternion;
//    this->Alvar::frame_id = alvar.frame_id;
//    this->type = FilterObjectType::alvar;
//  }


  virtual double idMatchingCost(const FilterObject& other) = 0;
  //  virtual bool idIsConfident();
  //  virtual void mergeWithInputObject(const FilterObject& o);

};



struct Cluster:FilterObject {
//  Cluster() {}
  Cluster(arr mean,
          arr points,
          std::string frame_id):
      mean(mean),
      points(points),
      frame_id(frame_id)
  {}
  arr mean;
  arr points;
  std::string frame_id;

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=cluster) return -1.;
    return length(mean - dynamic_cast<const Cluster*>(&other)->mean);
  }

};

struct Alvar:FilterObject {
//  Alvar() {}
  Alvar(arr position,
       ors::Quaternion quaternion,
       std::string frame_id):
      position(position),
      quaternion(quaternion),
      frame_id(frame_id)
  {}
  arr position;
  ors::Quaternion quaternion;
  std::string frame_id;

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=alvar) return -1.;
    return length(position - dynamic_cast<const Alvar*>(&other)->position);
  }

};

//struct Plane:FilterObject {
////  Plane() {}
//  Plane(arr mean,
//        arr points,
//        std::string frame_id):
//      mean(mean),
//      points(points),
//      frame_id(frame_id)
//  {}
//  arr mean;
//  arr points;
//  std::string frame_id;
//};




typedef mlr::Array<FilterObject*> FilterObjects;


