#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <gtest/gtest.h>
#include <functional>
#include <chrono>

#include <car_kinematic.h>

using namespace std;



/////////////////////////////////////
TEST(KOMO_realtime, zig_zag)
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
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

