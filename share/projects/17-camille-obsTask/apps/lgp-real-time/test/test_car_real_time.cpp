#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <gtest/gtest.h>
#include <functional>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <car_kinematic.h>
#include <velocity.h>

using namespace std;

struct CircularCage:TaskMap{

  CircularCage( const std::string & object, const arr & center, double radius )
    : object_( object )
    , center_( center )
    , radius_( radius )
  {
//    cv::Mat sensor_map = cv::imread("data/sensor_map.png");
//    cv::Mat bw;
//    cv::cvtColor(sensor_map, bw, CV_RG )
//    cv::Mat dist;
//    cv::distanceTransform(sensor_map, dist, 2, 3);
//    cv::imshow("dist", dist);
  }

  virtual mlr::String shortTag(const mlr::KinematicWorld& G)
  {
    return mlr::String("CircularCage");
  }

  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1) override
  {
    mlr::Frame *object = G.getFrameByName(object_.c_str());

    arr pos, Jpos;
    G.kinematicsPos(pos, Jpos, object);

    y.resize(1);//zeros(dim_phi(Gs, t));

    //y(0) = pos(0) - 1;
    y(0) = sqrt(pow(pos(0)-center_(0), 2.0) + pow(pos(1)-center_(1), 2.0)) - 2.0;

    if(&J)
    {
      J = zeros(dim_, Jpos.d1);
      auto theta = std::atan2(pos(1)-center_(1), pos(0)-center_(0));

      J(0, 0) = cos(theta) * Jpos(0, 0);
      J(0, 1) = sin(theta) * Jpos(1, 1);
    }
  }

  virtual uint dim_phi(const mlr::KinematicWorld& K) override
  {
    return dim_;
  }

private:
  static const uint dim_ = 1;
  std::string object_;
  double radius_;
  arr center_;
};


/////////////////////////////////////
void init_from_pose(double x, double y, double yaw,  KOMO & komo)
{
  komo.world.q(0)=x;
  komo.world.q(1)=y;
  komo.world.q(2)=yaw;
  komo.world.calc_Q_from_q();
  komo.world.calc_fwdPropagateFrames();
  komo.reset();
}

void init_with_last_traj(double x, double y, double yaw, const arr & last_x, KOMO & komo)
{
  if(!last_x.d0)
  {
    init_from_pose(x, y, yaw, komo);
    return;
  }

  // init prefix
  komo.world.q(0)=x;
  komo.world.q(1)=y;
  komo.world.q(2)=yaw;
  komo.world.calc_Q_from_q();
  komo.world.calc_fwdPropagateFrames();

  // init x
  auto new_x = last_x;

//  const uint n_steps_ahead = 0;
//  for(auto s=0; s < last_x.d0 - n_steps_ahead*komo.world.q.d0; ++s)
//  {
//    new_x(s) = last_x(s+n_steps_ahead*komo.world.q.d0);
//  }

  komo.set_x(new_x);
  komo.reset();

//  komo.configurations(5)->watch(true);
}

TEST(KOMO_realtime, box)
{
  mlr::KinematicWorld kin( "data/LGP-real-time.g" );
  const uint n_steps = 5;

  // run
  double x = 0;
  double y = 0;
  double yaw = 0;
  double elapsed_t = 1.0 / n_steps;
  double total_opt_time = 0.0;
  arr last_x;
  for(auto i = 0; i < 20; ++i)
  {
    // create komo
    KOMO komo;
    komo.setModel( kin, false, false, true, false, false );
    komo.setTiming(3.0, n_steps, 1);

    // speed
    komo.setSquaredQAccelerations();
    komo.setTask(0, -1, new Velocity("car_ego", 0.5), OT_sumOfSqr, NoArr, 1e2, 1);
    komo.setTask(0, -1, new CarKinematic("car_ego"), OT_eq, NoArr, 1e2, 1);
    komo.setTask(0, -1, new CircularCage("car_ego", {0.5, -0.2}, 2.5), OT_ineq, NoArr, 1e2, 1);

    // update kin
    //init_from_pose(x, y, yaw, komo);
    init_with_last_traj(x, y, yaw, last_x, komo);

    komo.world.watch();
    //komo.checkGradients();

    // run
    auto start = std::chrono::high_resolution_clock::now();
    komo.run();
    auto end = std::chrono::high_resolution_clock::now();
    auto execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    total_opt_time += execution_time_us;

    std::cout << "execution time (ms):" << execution_time_us / 1000 << std::endl;

    //komo.getReport(true);
    //komo.displayTrajectory(0.1, true);
    last_x = komo.x;
    x = komo.configurations(2)->q(0);
    y = komo.configurations(2)->q(1);
    yaw = komo.configurations(2)->q(2);
    elapsed_t += 1.0 / n_steps;
  }

  std::cout << "total optimization time (ms):" << total_opt_time / 1000 << std::endl;
}


/////////////////////////////////////
/*TEST(KOMO_realtime, zig_zag)
{
  mlr::KinematicWorld kin( "data/LGP-real-time.g" );  
  const uint n_steps = 5;

  // run
  double x = 0;
  double y = 0;
  double yaw = 0;
  double elapsed_t = 1.0 / n_steps;
  for(auto i = 0; i < 20; ++i)
  {
      // create komo
      KOMO komo;
      komo.setModel( kin, true, false, true, false, false );

      komo.setTiming(4.0, n_steps, 1);

      // speed
      komo.setSquaredQAccelerations();
      komo.setVelocity( std::max(0.0, -elapsed_t)   , std::max(0.0, 1.0 -elapsed_t), "car_ego", NULL, OT_sumOfSqr, { 1.0, 0.5, 0 } );
      komo.setVelocity( std::max(0.0, 1.0-elapsed_t), std::max(0.0, 2.0 -elapsed_t), "car_ego", NULL, OT_sumOfSqr, { 1.0, -0.5, 0 } );
      komo.setVelocity( std::max(0.0, 2.0 -elapsed_t), std::max(0.0, 3.0 -elapsed_t), "car_ego", NULL, OT_sumOfSqr, { 1.0, 0.5, 0 } );
      komo.setVelocity( std::max(0.0, 3.0 -elapsed_t), -1.0, "car_ego", NULL, OT_sumOfSqr, { 1.0, -0.5, 0 } );

      komo.setTask(0, -1, new CarKinematic("car_ego"), OT_eq, NoArr, 1e2, 1);

      // update kin
      komo.world.q(0)=x;
      komo.world.q(1)=y;
      komo.world.q(2)=yaw;
      komo.world.calc_Q_from_q();
      komo.world.calc_fwdPropagateFrames();
      komo.reset();
      komo.world.watch();

      // run
      auto start = std::chrono::high_resolution_clock::now();
      komo.run();
      auto end = std::chrono::high_resolution_clock::now();
      auto execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

      std::cout << "execution time (ms):" << execution_time_us / 1000 << std::endl;

      x = komo.configurations(2)->q(0);
      y = komo.configurations(2)->q(1);
      yaw = komo.configurations(2)->q(2);
      elapsed_t += 1.0 / n_steps;
   }
}*/

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

