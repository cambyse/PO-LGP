#include "drake.h"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/primitives/signal_logger.h>

struct sMyIIWA{
    drake::systems::DiagramBuilder<double> builder;
    std::unique_ptr<drake::lcm::DrakeLcm> LCM_interface; //IPC with visualizer
    //three systems that are passed to the builder
    drake::systems::RigidBodyPlant<double>* kukaPlant = NULL;
    drake::systems::DrakeVisualizer* visualizer = NULL;
    drake::systems::SignalLogger<double>* logger = NULL;
    drake::systems::controllers::InverseDynamicsController<double>* controller = NULL;

    std::unique_ptr<drake::systems::Diagram<double>> system;

    std::vector<drake::MatrixX<double>> splineKnots;
};
