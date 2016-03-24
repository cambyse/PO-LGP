#pragma once
#include <Core/array.h>

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




struct FilterObject : Cluster, Plane {
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


