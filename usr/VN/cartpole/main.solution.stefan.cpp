/*
 * Balace a cart pole.
 *
 * - Use a PD controller
 * - do random sampling so find initial feasible values for the PD controller
 * - use a simple hill climber to optimize the parameters of the PD controller
 *
 * TODO don't open the OpenGL window if it's not needed
 */

#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>

// ============================================================================
struct CartPoleState;
void drawEnv(void *);
void glDrawCartPole(void *classP);

// ============================================================================
//Normalizes any number to an arbitrary range
//by assuming the range wraps around when going below min or above max
double normalise( const double value, const double start=-MT_PI, const double end=MT_PI) {
  const double width = end - start;
  const double offsetValue = value - start ;   // value relative to 0
  return (offsetValue - (floor(offsetValue / width) * width)) + start;
  // + start to reset back to start of original range
}
// ============================================================================
struct CartPoleState {
  OpenGL gl;

  /// @name state
  double x;        /// pos
  double xdot;     /// velocity
  double theta;    /// angle
  double thetadot; /// angular velocity

  /// @name constants
  const double noise;
  const double tau; /// legth of a time step
  const double g;   /// gravitational constant
  const double l;   /// length of the pendulum
  const double mp;  /// mass of the pendulum
  const double mc;  /// mass of the car
  const double c1;
  const double c2;

  CartPoleState(double newNoise=.005)
      : x(0.)
      , xdot(0.)
      // , theta(.3)
      , theta(MT_PI/3.)
      , thetadot(0.)
      , noise(newNoise)
      , tau(1./60.)
      , g(9.81)
      , l(1.)
      , mp(1.)
      , mc(1.)
      , c1(1. / (mp + mc))
      , c2(l * mp * (1. / (mp + mc)))
  {
    gl.add(drawEnv, this);
    gl.add(glDrawCartPole, this);
    gl.camera.setPosition(10., -50., 10.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
    gl.update();
  }

  void step(double u, bool verbose=false) {
    double sint = sin(theta);
    double cost = cos(theta);
    double thetaddot =
      (g * sint + cost * (-c1 * u - c2 * thetadot * thetadot * sint)) /
      (4/3. *  l - c2 * cost * cost);
    double xddot = c1 * u + c2 * (thetadot * thetadot * sint - thetaddot * cost);

    // simple Euler integration
    x        += xdot * tau;
    xdot     += xddot * tau;
    theta    += thetadot * tau;
    thetadot += thetaddot * tau;

    theta = normalise(theta);

    // add noise
    // x += noise * rnd.gauss();
    xdot += noise * rnd.gauss();
    // theta += noise * rnd.gauss();
    thetadot += noise * rnd.gauss();

    if (verbose) {
      cout << "control: " << u << endl;
      cout << "linear:  " << x << " " << xdot << " " << xddot << endl;
      cout << "angular: " << theta << " " << thetadot << " " << thetaddot << endl;
      cout << "error -- pos: " << x << " angle:" << theta << endl;
      cout << "-------------------------------------------------" << endl;
    }
  }
};

// ============================================================================
void glDrawCartPole(void *classP) {
  CartPoleState *s = (CartPoleState *)classP;
  double GLmatrix[16];
  ors::Transformation f;
  f.setZero();
  // cart
  f.addRelativeTranslation(s->x, 0., 1.);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8, .2, .2);
  glDrawBox(1., .2, .2);
  // pole
  f.addRelativeRotationRad(s->theta, 0., 1., 0.);
  f.addRelativeTranslation(0., 0., .5);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.2, .2, .2);
  glDrawBox(.1, .1, 1.);
}

double getControl(const CartPoleState& cartPole, const arr& weight) {
  double u = weight(0) * cartPole.x +
             weight(1) * cartPole.xdot +
             weight(2) * cartPole.theta +
             weight(3) * cartPole.thetadot;
  return u;
}


/**
 * Run the cartPole and return the quared error for the given controller.
 */
double run(const arr& weight, bool visualize) {
  CartPoleState cartPole;
  double err = 0;
  for (int t = 0; t < 1000; ++t) {
    double u = getControl(cartPole, weight);
    cartPole.step(u);
    // err += (cartPole.x * cartPole.x +
    //         cartPole.theta * cartPole.theta);
    err += (cartPole.x * cartPole.x +
            cartPole.xdot * cartPole.xdot +
            cartPole.theta * cartPole.theta +
            cartPole.thetadot * cartPole.thetadot);

    if (visualize)
      cartPole.gl.update();
  }
  return err;
}


/**
 * @brief Optimize parameters.
 *
 * This is a simple hill climber.
 *
 * - We check each parameter dimension separately.
 * - If a step decreases our error we increase the step size.
 *   Note that we try steps in both directions:
 *   weight(i) += stepsize(i)
 *   weight(i) -= stepsize(i)
 *
 * - If not we decrease our step size and try again.
 *
 * @param weight initial weight
 * @param noise enable noise
 *
 * @return the optimized weights
 */
arr optimize(const arr& weight) {
  // init some stuff
  arr step_size = {1., 1., 1., 1.};
  step_size *= 0.1;
  double err = 999999999999;
  double best_err = run(weight, false);
  uint no_weigth_changes = 0;

  // optimize
  int max_trials = 50;
  for (int trial = 0; trial < max_trials; ++trial) {
    cout << trial << " -- err " << best_err << " weight: " << weight << endl;
    arr prev_best_weight = weight;

    // iterate over the parameters
    for (int i = 0; i < weight.N; ++i) {

      // change weights and run
      weight(i) += step_size(i);
      err = run(weight, false);
      // success --> right direction
      if (err < best_err) {
        best_err = err;
        step_size(i) *= 1.1;
        continue; // continue to update different parameter
      }

      // no success --> try different direction
      weight(i) -= (2 * step_size(i));
      err = run(weight, false);
      if (err < best_err) {
        best_err = err;
        step_size(i) *= 1.1;
        continue; // continue to update different parameter
      }

      // neither side was successful --> undo weight change & decrease the
      // step_size
      weight(i) += step_size(i);
      step_size(i) += 0.9;
    }

    // abort when bored
    if (prev_best_weight == weight) {
        no_weigth_changes++;
        cout << "Weights did not change" << endl;
      if (no_weigth_changes > 2) {
        cout << "Finish optimization: no change 3 times in a row." << endl;
        return weight;
      }
    } else {
      no_weigth_changes = 0;
    }
  } // end outer loop
  cout << "Finish optimization: max iteration reached." << endl;
  return weight;
}

// ============================================================================
arr getStartingPoints() {
  cout << "sampling starting parameters" << endl;
  arr starting_points;
  arr tmpw = zeros(500, 4);
  rndUniform(tmpw, 0, 100, false);
  for (int i = 0; i < 500; ++i) {
    auto w = tmpw.row(i);
    w.reshape(4);
    double err = run(w, false);
    // cout << i << " testing " << w << " err " << err << endl;

    // if err is not NaN we can use the parameter as initialization
    if (err == err)
      starting_points.append(w.resize(4));
  }
  starting_points.reshape(starting_points.N/4, 4);
  cout << starting_points.d0 << " feasible parameters found" << endl;
  return starting_points;
}

// ============================================================================
int main(int argc, char **argv) {
  arr parameters = getStartingPoints();

  for (int i = 0; i < parameters.d0; ++i) {
    // initial weights
    arr initial_weight = parameters[i];
    double err = run(initial_weight, false);
    cout << "before optimization " << initial_weight
         << "err " << err
         << endl;

    // optimize the weights
    arr weight_optimized = optimize(initial_weight);
    err = run(weight_optimized, true);
    cout << "after optimization " << weight_optimized
         << "err " << err
         << endl;
  }

  return 0;
}
