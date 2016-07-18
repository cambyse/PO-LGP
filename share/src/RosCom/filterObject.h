#pragma once
#include <Core/array.h>
#include <Ors/ors.h>

struct FilterObject {
  enum FilterObjectType { cluster, plane, alvar, optitrackmarker, optitrackbody };

  const std::string type_name()
  {
    return m.at(this->type);
  }

  uint id;
  double relevance = 1;
  ors::Transformation transform;
  ors::Transformation frame;
  FilterObjectType type;
  FilterObject(){
    transform.setZero();
    frame.setZero();
  }
  virtual ~FilterObject(){}

  virtual double idMatchingCost(const FilterObject& other) = 0;
  virtual void write(ostream& os) const{
    os <<" trans=" <<transform <<" frame=" <<frame;
  }

private:
  std::map<FilterObjectType, std::string> m = {
    std::make_pair(FilterObjectType::cluster, "cluster"),
    std::make_pair(FilterObjectType::alvar, "alvar"),
    std::make_pair(FilterObjectType::plane, "plane"),
    std::make_pair(FilterObjectType::optitrackmarker, "optitrack_marker"),
    std::make_pair(FilterObjectType::optitrackbody, "optitrack_body")
  };

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
    ors::Vector diff = (this->frame * ors::Vector(this->mean)) -
                      (dynamic_cast<const Cluster*>(&other)->frame * ors::Vector(dynamic_cast<const Cluster*>(&other)->mean));
    return diff.length();
  }

  virtual void write(ostream& os) const{
    os <<"cluster_" <<id <<": mean=" <<mean;
    FilterObject::write(os);
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
    ors::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const Alvar*>(&other)->frame * dynamic_cast<const Alvar*>(&other)->transform.pos);
    return dist.length();
  }

//  virtual void write(ostream& os) const{
//    os <<"alvar" <<id <<": mean=" <<mean;
//    FilterObject::write(os);
//  }
};



struct OptitrackMarker:FilterObject {
  std::string frame_id;

  OptitrackMarker(std::string frame_id):
      frame_id(frame_id)
  {
    this->type = FilterObjectType::optitrackmarker;
  }

  OptitrackMarker(const OptitrackMarker &obj)
  {
    this->frame_id = obj.frame_id;
    this->type = obj.type;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=optitrackmarker) return -1.;
    ors::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackMarker*>(&other)->frame * dynamic_cast<const OptitrackMarker*>(&other)->transform.pos);
    return dist.length();
  }

//  virtual void write(ostream& os) const{
//    os <<"alvar" <<id <<": mean=" <<mean;
//    FilterObject::write(os);
//  }
};

struct OptitrackBody:FilterObject {
  std::string frame_id;

  OptitrackBody(std::string frame_id):
      frame_id(frame_id)
  {
    this->type = FilterObjectType::optitrackbody;
  }

  OptitrackBody(const OptitrackBody &obj)
  {
    this->frame_id = obj.frame_id;
    this->type = obj.type;
    this->relevance = obj.relevance;
    this->id = obj.id;
    this->transform = obj.transform;
    this->frame = obj.frame;
  }

  virtual double idMatchingCost(const FilterObject& other){
    if(other.type!=optitrackbody) return -1.;
    ors::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const OptitrackBody*>(&other)->frame * dynamic_cast<const OptitrackBody*>(&other)->transform.pos);
    return dist.length();
  }

//  virtual void write(ostream& os) const{
//    os <<"alvar" <<id <<": mean=" <<mean;
//    FilterObject::write(os);
//  }
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


