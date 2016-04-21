#pragma once
#include <Core/array.h>
#include <Ors/ors.h>

struct FilterObject {
  enum FilterObjectType { cluster, plane, alvar };

  const std::string type_name()
  {
    return m.at(this->type);
  }

  uint id;
  double relevance = 1;
  ors::Transformation transform;
  ors::Transformation frame;
  FilterObjectType type;
  FilterObject(){}
  virtual ~FilterObject(){}

  virtual double idMatchingCost(const FilterObject& other) = 0;

private:
  std::map<FilterObjectType, std::string> m = {
    std::make_pair(FilterObjectType::cluster, "cluster"),
    std::make_pair(FilterObjectType::alvar, "alvar"),
    std::make_pair(FilterObjectType::plane, "plane")
  };

  //  virtual bool idIsConfident();
  //  virtual void mergeWithInputObject(const FilterObject& o);

};



struct Cluster:FilterObject {
  arr mean;
  arr points;
  std::string frame_id;

  Cluster(arr mean,
          arr points,
          std::string frame_id):
      mean(mean),
      points(points),
      frame_id(frame_id)
  {
    this->type = FilterObjectType::cluster;
  }

  Cluster(const Cluster &obj)
  {
    this->frame_id = obj.frame_id;
    this->mean = obj.mean;
    this->points = obj.points;
    this->type = obj.type;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=cluster) return -1.;
    return length(this->mean - dynamic_cast<const Cluster*>(&other)->mean);
  }

};

struct Alvar:FilterObject {
  std::string frame_id;

  Alvar(std::string frame_id):
      frame_id(frame_id)
  {
    this->type = FilterObjectType::alvar;
  }

  Alvar(const Alvar &obj)
  {
    this->frame_id = obj.frame_id;
    this->type = obj.type;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=alvar) return -1.;
    ors::Vector dist = this->transform.pos - dynamic_cast<const Alvar*>(&other)->transform.pos;
    return dist.length();
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


