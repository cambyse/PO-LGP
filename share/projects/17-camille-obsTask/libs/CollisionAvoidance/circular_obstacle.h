#pragma once

#include <KOMO/komo.h>


struct CircularObstacle:Feature{

  CircularObstacle( const std::string & agent_object, const arr & position, double radius, double safety_distance = 0.5 )
    : agent_object_( agent_object )
    , position_( position )
    , radius_( radius )
    , safety_distance_( safety_distance )
    , car_circle_radius_(2.15)
    , circle_relative_position_(1.25, 0, 0)
  {
  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("CarObstacle");
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) override
  {
    rai::Frame *object = G.getFrameByName(agent_object_.c_str());
    // init
    y.resize(1);//zeros(dim_phi(Gs, t));
    y(0) = 0;
    if(&J)
    {
      J = zeros(dim_, G.q.d0);
    }

    // compute cost
    punctual_phi(object, circle_relative_position_, y, J, G);
  }

  void punctual_phi(rai::Frame * object, const rai::Vector & rel, arr& y, arr& J, const rai::KinematicWorld& G) const
  {
      // compute cost
      arr pos, Jpos;
      G.kinematicsPos(pos, Jpos, object, rel);

      const auto radius = sqrt(pow(pos(0)-position_(0), 2.0) + pow(pos(1)-position_(1), 2.0));
      y(0) += - radius + radius_ + car_circle_radius_ + safety_distance_;

      //std::cout << "pos " << pos(0) << " " << pos(1) << std::endl;
      //std::cout << "y(0)" << y(0) << std::endl;

      if(&J)
      {
        auto theta = std::atan2(pos(1)-position_(1), pos(0)-position_(0));

        J(0, 0) += - cos(theta) * Jpos(0, 0);
        J(0, 1) += - sin(theta) * Jpos(1, 1);
      }
  }

  virtual uint dim_phi(const rai::KinematicWorld& K) override
  {
    return dim_;
  }

  void set_obstacle_position(const arr & position)
  {
      position_ = position;
  }

private:
  static const uint dim_ = 1;
  std::string agent_object_;
  double radius_;
  arr position_;
  double safety_distance_;
  rai::Vector circle_relative_position_;
  const double car_circle_radius_;
};



struct Car3CirclesCircularObstacle:Feature{

  Car3CirclesCircularObstacle( const std::string & agent_object, const arr & position, double radius, double safety_distance = 0.5 )
    : agent_object_( agent_object )
    , position_( position )
    , radius_( radius )
    , safety_distance_( safety_distance )
    , car_circle_radius_(1.5)
//    , circle_relative_position_(1.25, 0, 0)
  {
      circle_relative_positions_[0] = (1.25, 0, 0);
      circle_relative_positions_[1] = (-0.9, 0, 0);
      circle_relative_positions_[2] = (3.4, 0, 0);
  }

  virtual rai::String shortTag(const rai::KinematicWorld& G)
  {
    return rai::String("Car3CirclesCircularObstacle");
  }

  virtual void phi(arr& y, arr& J, const rai::KinematicWorld& G) override
  {
    rai::Frame *object = G.getFrameByName(agent_object_.c_str());
    // init
    y.resize(dim_);//zeros(dim_phi(Gs, t));
    y(0) = 0;
    if(&J)
    {
      J = zeros(dim_, G.q.d0);
    }

    // compute cost
    for(auto i = 0; i < dim_; ++i)
    {
        punctual_phi(object, circle_relative_positions_[i], i, y, J, G);
    }
  }

  void punctual_phi(rai::Frame * object, const rai::Vector & rel, std::size_t i, arr& y, arr& J, const rai::KinematicWorld& G) const
  {
      // compute cost
      arr pos, Jpos;
      G.kinematicsPos(pos, Jpos, object, rel);

      const auto radius = sqrt(pow(pos(0)-position_(0), 2.0) + pow(pos(1)-position_(1), 2.0));
      y(i) = - radius + radius_ + car_circle_radius_ + safety_distance_;

      //std::cout << "pos " << pos(0) << " " << pos(1) << std::endl;
      //std::cout << "y(0)" << y(0) << std::endl;

      if(&J)
      {
        auto theta = std::atan2(pos(1)-position_(1), pos(0)-position_(0));

        J(i, 0) = - cos(theta) * Jpos(0, 0);
        J(i, 1) = - sin(theta) * Jpos(1, 1);
      }
  }

  virtual uint dim_phi(const rai::KinematicWorld& K) override
  {
    return dim_;
  }

  void set_obstacle_position(const arr & position)
  {
      position_ = position;
  }

private:
  static constexpr uint dim_ = 3;
  std::string agent_object_;
  double radius_;
  arr position_;
  double safety_distance_;
  std::array<rai::Vector, dim_> circle_relative_positions_;
  const double car_circle_radius_;
};
