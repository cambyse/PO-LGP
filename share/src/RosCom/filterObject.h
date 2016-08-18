#pragma once
#include <Core/array.h>
#include <Ors/ors.h>

struct FilterObject {
  enum FilterObjectType { cluster, plane, alvar };

  const std::string type_name() { return m.at(this->type); }

  uint id;
  double relevance = 1;
  ors::Transformation transform;
  ors::Transformation frame;
  FilterObjectType type;
  FilterObject();
  virtual ~FilterObject(){}

  virtual double idMatchingCost(const FilterObject& other) = 0;
  virtual void write(ostream& os) const;

private:
  std::map<FilterObjectType, std::string> m = {
    std::make_pair(FilterObjectType::cluster, "cluster"),
    std::make_pair(FilterObjectType::alvar, "alvar"),
    std::make_pair(FilterObjectType::plane, "plane")
  };

};

struct Cluster:FilterObject {
  arr mean;
  arr points;
  std::string frame_id;

  Cluster(arr mean, arr points, std::string frame_id);
  Cluster(const Cluster &obj);
  virtual double idMatchingCost(const FilterObject& other);
  virtual void write(ostream& os) const;
};

struct Plane:FilterObject {
  arr normal;
  arr center; // not really necesarily center - but at least a point on the plane
  arr hull;
  std::string frame_id;

  Plane(arr normal, arr center, arr hull, std::string frame_id);
  Plane(const Plane &obj);
  virtual double idMatchingCost(const FilterObject& other);
  virtual void write(ostream& os) const;
};

struct Alvar:FilterObject {
  std::string frame_id;

  Alvar(std::string frame_id);
  Alvar(const Alvar &obj);
  virtual double idMatchingCost(const FilterObject& other);
  virtual void write(ostream& os) const;
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


