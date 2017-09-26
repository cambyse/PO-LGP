#include <drake/common/find_resource.h>
#include <drake/common/text_logging_gflags.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_polynomial_trajectory.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/primitives/signal_logger.h>

#include <random>

#include "drake.h"
#include "drake-internal.h"

double realtime_rate = 1.0; // "Rate at which to run the simulation, relative to realtime"
double simulation_time = 2.0; //"How long to simulate"

Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> conv_arr2eigen(const arr& X){
  if(X.nd==2) return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(X.p, X.d0, X.d1);
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(X.p, X.d0, 1);
}

arr conv_eigen2arr(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& X){
  arr x(X.data(), X.size());
  if(X.IsRowMajor) return x.reshape(X.rows(), X.cols()); //by reference!
  x.reshape(X.cols(), X.rows());
  return ~x; //by copy!

//  arr x(X.rows(), X.cols());
//  for(uint i=0;i<x.d0;i++) for(uint j=0;j<x.d1;j++) x(i,j) = X(i,j);
//  return x;
//  return .reshape(X.rows(), X.cols());
}

MyIIWA::MyIIWA(int argc, char *argv[]){
  s = new sMyIIWA;
  gflags::SetUsageMessage(
        "Simulates the Kuka iiwa arm tracking a trajectory with an inverse "
        "dynamics controller.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  // Instantiate LCM interface and start receiving. This is needed for
  // communicating with drake-visualizer
  s->LCM_interface = std::make_unique<drake::lcm::DrakeLcm>();
  s->LCM_interface->StartReceiveThread();
}

MyIIWA::~MyIIWA(){
  delete s;
}

void MyIIWA::addKukaPlant(){
  // Load the Kuka model from a URDF-file
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        "/opt/drake/share/drake/drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf",
        drake::multibody::joints::kFixed, iiwa.get());

  // Add the Kuka arm to the diagram. The RigidBodyPlant block has one input
  // (actuator torques) and one output (joint state).
  s->kukaPlant = s->builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(iiwa));

  // Add a visualizer to the diagram and connect its input to the output of the
  // plant.
  s->visualizer = s->builder.AddSystem<drake::systems::DrakeVisualizer>(
        s->kukaPlant->get_rigid_body_tree(), s->LCM_interface.get());
  s->builder.Connect(s->kukaPlant->get_output_port(0), s->visualizer->get_input_port(0));
}

void MyIIWA::addController(){
  const int kNumPositions{s->kukaPlant->get_num_positions()};

  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  drake::VectorX<double> kp{100 * drake::VectorX<double>::Ones(kNumPositions)};
  drake::VectorX<double> kd{2 * kp.cwiseSqrt()};
  drake::VectorX<double> ki{drake::VectorX<double>::Ones(kNumPositions)};
  s->controller = s->builder.AddSystem<
      drake::systems::controllers::InverseDynamicsController<double>>(s->kukaPlant->get_rigid_body_tree().Clone(), kp, ki, kd,
                                                                      true /*has_reference_acceleration*/);

  // Connect the controller's output to the plant's input
  drake::log()->debug("Connecting controller output to plant input");
  s->builder.Cascade(*s->controller, *s->kukaPlant);

  // State feedback from the plant to the controller.
  drake::log()->debug(
        "Connecting plant output to controller estimated state input");
  s->builder.Connect(s->kukaPlant->get_output_port(0),
                  s->controller->get_input_port_estimated_state());
}

void MyIIWA::addLogger(){
  const int kNumPositions{s->kukaPlant->get_num_positions()};
  const int kNumVelocities{s->kukaPlant->get_num_velocities()};

  s->logger = s->builder.AddSystem<drake::systems::SignalLogger<double>>(kNumPositions+kNumVelocities);

  s->builder.Connect(s->kukaPlant->get_output_port(0),
                     s->logger->get_input_port());
}

void MyIIWA::addReferenceTrajectory(const arr& knots){
  std::default_random_engine random_generator{1234};
  std::vector<double> t_values;

  if(&knots){
    const double dt = simulation_time / (knots.d0 - 1);
    for(uint i=0; i<knots.d0; i++) {
      t_values.push_back(i * dt);
      s->splineKnots.push_back( conv_arr2eigen(knots[i]) );
    }
  }else{
    const int kNumKnots = 5;
    const double dt = simulation_time / (kNumKnots - 1);
    for (int i = 0; i < kNumKnots; ++i) {
      t_values.push_back(i * dt);
      s->splineKnots.push_back(
            s->kukaPlant->get_rigid_body_tree().getRandomConfiguration(random_generator));
    }
  }

  drake::PiecewisePolynomialTrajectory reference_position_trajectory{
    PiecewisePolynomial<double>::Pchip(t_values, s->splineKnots,
                                       true /*zero_end_point_derivatives*/)};

  // This reference trajectory can used in the diagram by means of
  // TrajectorySource blocks.
  auto reference_state_source =
      s->builder.AddSystem<drake::systems::TrajectorySource<double>>(reference_position_trajectory, 1 /*output_derivative_order*/);
  auto reference_acceleration_source =
      s->builder.AddSystem<drake::systems::TrajectorySource<double>>(*reference_position_trajectory.derivative(2));

  // Finally, we can connect the trajectories to the inputs of the controller.
  drake::log()->debug("Connecting desired state trajectory to controller");
  s->builder.Connect(reference_state_source->get_output_port(),
                     s->controller->get_input_port_desired_state());
  drake::log()->debug(
        "Connecting desired acceleration trajectory to controller");
  s->builder.Connect(reference_acceleration_source->get_output_port(),
                     s->controller->get_input_port_desired_acceleration());

}

void MyIIWA::simulate(){
  // Instantiate and configure simulator.
  drake::systems::Simulator<double> simulator{*s->system};
  simulator.set_target_realtime_rate(realtime_rate);
  for (int index = 0; index < s->kukaPlant->get_num_positions(); index++) {
    s->kukaPlant->set_position(simulator.get_mutable_context(), index,
                               s->splineKnots.front()(index));
  }
  simulator.Initialize();

  // Run simulation.
  simulator.StepTo(simulation_time);
}

arr MyIIWA::getLog(){
  return conv_eigen2arr(s->logger->data().transpose());
}

void MyIIWA::build(){
  s->system = s->builder.Build();
}
