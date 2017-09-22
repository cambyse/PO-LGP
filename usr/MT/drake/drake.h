#pragma once

#include <Core/array.h>

#include <random>

#include <drake/systems/framework/diagram_builder.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/primitives/signal_logger.h>

struct MyIIWA{
  drake::systems::DiagramBuilder<double> builder;
  std::unique_ptr<drake::lcm::DrakeLcm> LCM_interface; //IPC with visualizer
  //three systems that are passed to the builder
  drake::systems::RigidBodyPlant<double>* kukaPlant;
  drake::systems::DrakeVisualizer* visualizer;
  drake::systems::SignalLogger<double>* logger;
  drake::systems::controllers::InverseDynamicsController<double>* controller;

  std::unique_ptr<drake::systems::Diagram<double>> system;

  std::vector<drake::MatrixX<double>> splineKnots;

  MyIIWA(int argc, char* argv[]);

  void addKukaPlant();
  void addController();
  void addLogger();
  void addReferenceTrajectory(const arr& knots=NoArr);

  void build();

  void simulate();

  arr getLog();
};
