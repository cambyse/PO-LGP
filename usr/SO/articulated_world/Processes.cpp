#include "Processes.h"

// ============================================================================
// PERCEPTION
void FakePerceptionP::step() {
  // robot state
  ors::Body* agent = geometricState->get_ors().getBodyByName("robot");
  robot->set_pos(agent->X.pos, this);

  // percepts are all bodies in the wolrd (except the robot: N-1)
  uint N = geometricState->get_ors().bodies.N;
  percepts->objects.clear();
  /* cout << "number of percepts " << percepts->get_objects(this).N << endl; */
  /* cout << "all percepts " << percepts->get_objects(this) << endl; */
  for (uint i = 0; i < N; i++) {
    ors::Body* p = geometricState->get_ors().bodies(i);
    /* cout << "Potential percept " << p->index << endl; */
    if (agent->index != p->index) {
      percepts->objects.append(&p->X.pos);
    }
  }
};

// ============================================================================
// COGNITION
void CognitionP::step() {
  if (worldState->changed) {
    // wait :)
    movementRequest->control_u = zeroControl;
  } else {
    // select a destination from the percepts
    ::srand(time(NULL));
    uint N = percepts->objects.N;
    if (N > 0) {
      ors::Vector destination = *percepts->objects(::rand() % N);
      ors::Vector control = destination - robotPos->pos;

      cout << "Destination    " << destination   << endl;
      cout << "Robot pos      " << robotPos->pos << endl;
      cout << "Control        " << control       << endl;

      double Kp = 0.01;
      control.x = mlr::sign(control.x) * Kp;
      control.y = mlr::sign(control.y) * Kp;
      control.z = 0 * mlr::sign(control.z) * Kp;

      cout << "Control scaled " << control       << endl;
      cout << endl;

      movementRequest->control_u = control;
    } else {
      cout << "Too few (" << N << ") objects in the world" << endl;
    }
  }
}


// ============================================================================
// WorldStateProvider
void WorldStateProvider::step() {
  arr current;
  geometricState->get_ors().getJointState(current);

  /* cout << "joint dim " << geometricState->get_ors().getJointStateDimension(true) << endl; */
  /* cout << "current " << current.N << " " << current << endl; */

  if (firstRun) {
    previous = current;
    worldState->changed = false;
    firstRun = false;
    cout << "IN firstRun" << endl;
    cout << "setting..." << previous << endl;
  }

  // if world is changed
  cout << "prev " << previous << endl;
  cout << "curr " << current  << endl;

  // did world change?
  worldState->changed = norm(previous - current) > 0.01;
  previous = current;
}
