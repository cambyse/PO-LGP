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
    mu = f(u);
    Sigma = F*Sigma*~F + R;
  }

  void correct(const arr& z) {
    this->updateG();
    arr K, v;
    K = Sigma*~G*inverse(G*Sigma*~G+Q);
    v = z - g();
    mu = mu + K*v;
    Sigma = (I-K*G)*Sigma;
  }

  /// Motion model
  arr f(arr u) {
    arr mf = zeros(3);
    mf(0) = mu(0) + u(0)*cos(mu(2));
    mf(1) = mu(1) + u(0)*sin(mu(2));
    mf(2) = mu(2) + u(0)/sim.L*tan(u(1));
    return mf;
  }

  /// Observation model
  arr g() {
    arr obs;
    sim.getMeanObservationAtState(obs, mu);
    return obs;
  }

  void updateF(arr u) {
    /*F = zeros(3,2);
    F(0,0) = cos(mu(2));
    F(0,1) = 0.0;
    F(1,0) = sin(mu(2));
    F(1,1) = 0.0;
    F(2,0) = tan(u(1))/sim.L;
    F(2,1) = u(0)/sim.L/cos(u(1))/cos(u(1));*/

    F = zeros(3,3);
    F(0,0) = 1.0;
    F(0,1) = 0.0;
    F(0,2) = -u(0)*sin(mu(2));
    F(1,0) = 0.0;
    F(1,1) = 1.0;
    F(1,2) = u(0)*cos(mu(2));
    F(2,0) = 0.0;
    F(2,1) = 0.0;
    F(2,2) = 1.0;

    /*
    F = zeros(3,5);
    F(0,0) = 1.0;
    F(0,1) = 0.0;
    F(0,2) = -u(0)*sin(mu(2));
    F(0,3) = cos(mu(2));
    F(0,4) = 0.0;
    F(1,0) = 0.0;
    F(1,1) = 1.0;
    F(1,2) = u(0)*cos(mu(2));
    F(1,3) = sin(mu(2));
    F(1,4) = 0.0;
    F(2,0) = 0.0;
    F(2,1) = 0.0;
    F(2,2) = 1.0;
    F(2,3) = tan(u(1))/sim.L;
    F(2,4) = u(0)/sim.L/cos(u(1))/cos(u(1));
    */
  }

  void updateG() {
    sim.getObservationJacobianAtState(G, mu);
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
  ekf.R = eye(3)*sim.dynamicsNoise;
  ekf.Q = eye(4)*sim.observationNoise;
  ekf.I = eye(3);

  sim.gl->watch();
  for (uint t=0;t<4000;t++) {
    mlr::wait(0.01);
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
