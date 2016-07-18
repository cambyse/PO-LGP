#include "filterObject.h"




FilterObject::FilterObject(){
  transform.setZero();
  frame.setZero();
}

void FilterObject::write(ostream& os) const{
  os <<" trans=" <<transform <<" frame=" <<frame;
}

//============================================================================

Cluster::Cluster(arr mean, arr points, std::string frame_id)
  : mean(mean),
    points(points),
    frame_id(frame_id) {
  this->type = FilterObjectType::cluster;
}


Cluster::Cluster(const Cluster& obj) {
  this->frame_id = obj.frame_id;
  this->mean = obj.mean;
  this->points = obj.points;
  this->type = obj.type;
  this->relevance = obj.relevance;
  this->id = obj.id;
  this->transform = obj.transform;
  this->frame = obj.frame;
}

double Cluster::idMatchingCost(const FilterObject& other){
  if(other.type!=cluster) return -1.;
  ors::Vector diff = (this->frame * ors::Vector(this->mean)) -
                     (dynamic_cast<const Cluster*>(&other)->frame * ors::Vector(dynamic_cast<const Cluster*>(&other)->mean));
  return diff.length();
}

void Cluster::write(ostream& os) const{
  os <<"cluster_" <<id <<": mean=" <<mean;
  FilterObject::write(os);
}

//============================================================================


Alvar::Alvar(std::string frame_id)
  : frame_id(frame_id) {
  this->type = FilterObjectType::alvar;
}


Alvar::Alvar(const Alvar& obj){
  this->frame_id = obj.frame_id;
  this->type = obj.type;
  this->relevance = obj.relevance;
  this->id = obj.id;
  this->transform = obj.transform;
  this->frame = obj.frame;
}

double Alvar::idMatchingCost(const FilterObject& other){
  if(other.type!=alvar) return -1.;
  ors::Vector dist = (this->frame * this->transform.pos) - (dynamic_cast<const Alvar*>(&other)->frame * dynamic_cast<const Alvar*>(&other)->transform.pos);
  return dist.length();
}

void Alvar::write(ostream& os) const{
  os <<"alvar_" <<id <<":";
  FilterObject::write(os);
}
