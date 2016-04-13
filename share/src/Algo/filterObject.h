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



struct Cluster {
  Cluster() {}
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

};

struct Alvar {
  Alvar() {}
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
};

struct Plane {
  Plane() {}
  Plane(arr mean,
        arr points,
        std::string frame_id):
      mean(mean),
      points(points),
      frame_id(frame_id)
  {}
  arr mean;
  arr points;
  std::string frame_id;
};




struct FilterObject : Cluster, Plane, Alvar {
  enum FilterObjectType { cluster, plane, alvar };

  uint id;
  double relevance;
  FilterObjectType type;

  FilterObject(){}

  FilterObject(Cluster cluster):FilterObject()
  {
    this->Cluster::mean = cluster.mean;
    this->Cluster::frame_id = cluster.frame_id;
    this->Cluster::points = cluster.points;
    this->type = FilterObjectType::cluster;
  }

  FilterObject(Alvar alvar):FilterObject()
  {
    this->Alvar::position = alvar.position;
    this->Alvar::quaternion = alvar.quaternion;
    this->Alvar::frame_id = alvar.frame_id;
    this->type = FilterObjectType::alvar;
  }


  double idMatchingCost(const FilterObject& other)
  {
    if (this->type != other.type)
    {
      NIY;
      exit(-1);
    }

    switch(this->type)
    {
      case FilterObjectType::cluster:
        return length(this->Cluster::mean - other.Cluster::mean);
        break;
      case FilterObjectType::alvar:
        return length(this->Alvar::position - other.Alvar::position);
        break;
      default:
        std::cout << "matching types: " << this->type << " cluster is: " << FilterObjectType::cluster << std::endl;
        NIY;
        exit(-1);
        break;
    }
  }

//  virtual bool idIsConfident();
//  virtual void mergeWithInputObject(const FilterObject& o);
};

typedef mlr::Array<FilterObject> FilterObjects;


