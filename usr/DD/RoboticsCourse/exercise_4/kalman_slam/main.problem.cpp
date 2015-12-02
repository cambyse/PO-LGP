#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>


// ============================================================================
// TODO resue the previous implementation as much as possible
class ExtendedKalmanFilter {
public:
  // mean and covariance
  arr mu, Sigma;
  // Jacobians
  arr F, G;
  // noise matrices
  arr R, Q;
  // Identity
  arr I;

  // We need the simulator for some calculations
  CarSimulator& sim;

  ExtendedKalmanFilter(CarSimulator& sim) : sim(sim) {}

  void predict(const arr& u) {
    this->updateF(u);
    // TODO resue the previous implementation as much as possible
  }

  void correct(const arr& z) {
    this->updateG();
    // TODO resue the previous implementation as much as possible
  }

  /// Motion model
  arr f(arr u) {
    // TODO implement the motion model
    return {};
  }

  /// Observation model
  arr g() {
    // TODO implement the observation model
    return {};
  }

  void updateF(arr u) {
    // TODO implement the jacobian F
    // F = ???
  }

  void updateG() {
    // TODO implement the Jacobian G
    // The simulator already give you the Jacobian
  }
};


// ============================================================================
int main(int argc,char **argv){
  CarSimulator sim;

  arr u = {.1, .2}; // fixed control signal
  arr z;
  sim.getRealNoisyObservation(z);

  ExtendedKalmanFilter ekf = ExtendedKalmanFilter(sim);
  // TODO what is mu and Sigma in the SLAM case?
  // ekf.mu = {0., 0., 0., ???};
  // ekf.Sigma = ???
  // TODO set the matrices R, Q, and I of the ekf

  sim.gl->watch();
  for (uint t=0;t<1000;t++) {
    sim.step(u);
    sim.getRealNoisyObservation(z);

    // EKF
    ekf.predict(u);
    ekf.correct(z);

    sim.gaussiansToDraw.resize(3);
    // TODO draw belief of robot pose
    // sim.gaussiansToDraw(0).a = ekf.mu.sub(0, 1);
    // sim.gaussiansToDraw(0).A = ekf.Sigma.sub(0, 1, 0, 1);
    // TODO draw belief of landmarks
    // sim.gaussiansToDraw(1).a = ???;
    // sim.gaussiansToDraw(1).A = ???;
    // sim.gaussiansToDraw(2).a = ???;
    // sim.gaussiansToDraw(2).A = ???;
  }

  return 0;
}
