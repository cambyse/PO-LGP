#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>


// ============================================================================
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
    // TODO write down the KF equation for the prediction
  }

  void correct(const arr& z) {
    this->updateG();
    // TODO write down the KF equation for the correction
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

  ExtendedKalmanFilter ekf = ExtendedKalmanFilter(sim);
  ekf.mu = {0., 0., 0.};
  ekf.Sigma = eye(3);
  // TODO set the matrices R, Q, and I of the ekf
  // sim.observationNoise (use when evaluating a particle likelihood)
  // sim.dynamicsNoise (use when propagating a particle)

  sim.gl->watch();
  for (uint t=0;t<1000;t++) {
    sim.step(u);
    sim.getRealNoisyObservation(z);

    // EKF
    ekf.predict(u);
    ekf.correct(z);

    // draw the belief
    sim.gaussiansToDraw.resize(1);
    sim.gaussiansToDraw(0).a = ekf.mu.sub(0, 1);
    sim.gaussiansToDraw(0).A = ekf.Sigma.sub(0, 1, 0, 1);

    arr tmp = ARR(sim.x, sim.y);
    cout << "err trans: " << length(ekf.mu.sub(0, 1) - tmp) << endl;;
  }

  return 0;
}
