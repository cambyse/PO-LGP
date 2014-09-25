#include <pr2/actionMachine.h>
#include <pr2/actions.h>

void testActionMachine()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  // auto a1 = activity.machine->add(
  //     new MoveEffTo("endeffR", ARR(.6, -.5, 1.2)));
  // activity.machine->waitForActionCompletion(a1);

  // auto a2 = activity.machine->add(
  //     new AlignEffTo("endeffR", ARR(1., 0., 0.), ARR(1., 0., 0.)));
  // activity.machine->waitForActionCompletion(a2);
  // auto a3 = activity.machine->add(new MoveEffTo("endeffL", ARR(.8, .4, 1.2)));

  // auto action = activity.machine->add(
  //     new MoveEffTo("endeffR", ARR(.6, -.4, 1.0))
  // );
  // activity.machine->waitForActionCompletion(action);

  // action = activity.machine->add(
  //     new AlignEffTo("endeffR", ARR(1, 0, 0), ARR(1, 0, 0)));
  GroundedAction* action = activity.machine->add(
      new PushForce("endeffR", ARR(1, 2, 3)/*, ARR(1, 4, 8)*/)
  );
  activity.machine->waitForActionCompletion(action);

  // MT::wait(2);

    // activity.machine->waitForActionCompletion(a3);

    // auto a4 = activity.machine->add(new PushForce("endeffR", ors::Vector(15, 0, 0), ARR(2, 0, 0)))
    // activity.machine->waitForActionCompletion(a4);
  // }
  engine().close(activity);
}


void do_the_dance()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  for (int i = 0; i < 5; ++i) {
    GroundedAction* a_right = activity.machine->add(new MoveEffTo("endeffR", {.6, -.5, 1.2}));
    GroundedAction* a_left = activity.machine->add(new MoveEffTo("endeffL", {.6, .6, 1.2}));
    activity.machine->waitForActionCompletion(a_left);
    activity.machine->waitForActionCompletion(a_right);

    GroundedAction* a_right2 = activity.machine->add(new MoveEffTo("endeffR", {.3, -.7, 1.0}));
    GroundedAction* a_left2 = activity.machine->add(new MoveEffTo("endeffL", {.3, .7, 1.0}));
    activity.machine->waitForActionCompletion(a_left2);
    activity.machine->waitForActionCompletion(a_right2);
  }

  engine().close(activity);
}


void test_push()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  GroundedAction* a_right = activity.machine->add(new MoveEffTo("endeffR", {.6, -.3, 1}));
  activity.machine->waitForActionCompletion(a_right);
  cout << "waiting" << endl;
  MT::wait(3);
  GroundedAction* push = activity.machine->add(new PushForce("endeffR", {.0, -.05, 0}/*, {0., 1., 0.}*/));
  activity.machine->waitForActionCompletion(push);

  engine().close(activity);
}


void idle()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  GroundedAction* right = activity.machine->add(new MoveEffTo("endeffR", {.95, -.2, .9}));
  GroundedAction* left = activity.machine->add(new MoveEffTo("endeffL", {.95, .2, .9}));
  activity.machine->waitForActionCompletion(right);
  activity.machine->waitForActionCompletion(left);

  // GroundedAction* align_right = activity.machine->add(new AlignEffTo("endeffR", {.95, 0, 0.}, {.95, 0, 0}));
  // GroundedAction* align_left = activity.machine->add(new AlignEffTo("endeffL", {.95, 0, 0.}, {.95, 0, 0}));
  // activity.machine->waitForActionCompletion(align_right);
  // activity.machine->waitForActionCompletion(align_left);

  engine().close(activity);
}

void test_collision()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);
  activity.machine->add(new MoveEffTo("endeffR", {.8, .1, .9}));
  activity.machine->add(new MoveEffTo("endeffL", {.8, -.1, .9}));
  activity.machine->waitForActionCompletion();
  cout << "actions done" << endl;
  MT::wait(5);
  engine().close(activity);
}

void idle2()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  auto t = activity.machine->add(new OrientationQuat("endeffR", {1, 1, 0, 0}));
  activity.machine->waitForActionCompletion(t);
  cout << "Done waiting" << endl;

  activity.machine->add(new MoveEffTo("endeffR", {.8, -.2, .9}));
//  activity.machine->add(new OrientationQuat("endeffR", {1, 1, 0, 0}));

  activity.machine->waitForActionCompletion();
  MT::wait(5);

  // activity.machine->add(new MoveEffTo("endeffR", {.6, -.2, .9}));
  // activity.machine->add(new AlignEffTo("endeffR", {1, 0, 0.}, {1, 0, 0}));
  // activity.machine->waitForActionCompletion();

  engine().close(activity);
}

void set_q()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

//   for (auto joint : activity.machine->s->world.joints)
//     cout << joint->qIndex << " " << joint->name << endl;

  int jointID = -(activity.machine->s->world.getJointByName("torso_lift_joint")->qIndex);
  cout << "joint: " << jointID << endl;
  activity.machine->add(new SetQ("XXX", jointID, 3));

  activity.machine->waitForActionCompletion();
  cout << "DONE" << endl;
  engine().close(activity);
}

// ============================================================================
struct UserInput {
  arr joint_pos;
  int check_joint_id;
};

UserInput get_input() {
  UserInput input;
  double tmp;

  cout << "Enter rot (degrees):" << endl;
  std::cin >> tmp;
  input.joint_pos.append((double)tmp);

  cout << "Enter pris (m):" << endl;
  std::cin >> tmp;
  input.joint_pos.append((double)tmp);

  cout << "Joint id to check" << endl;
  std::cin >> input.check_joint_id;

  cout << "done..." << endl;

  return input;
}

class IcraExperiment
{
public:
  IcraExperiment()
    :activity()
  {
    activity.machine->add(new CoreTasks());
    engine().open(activity);
  }

  virtual ~IcraExperiment()
  {
    engine().close(activity);
  }

  void run()
  {
    ors::Transformation pose; pose.setZero();
    MT::wait(1.);
    // align
    pose = activity.machine->s->MP.world.getShapeByName("endeffL")->X;
    //pose.pos.x += .2;
    //pose.pos.y += .1;
    //pose.pos.z -= .1;
    //pose.rot.setDeg(90, 1, 0 ,0);
    
    cout << "aligne.." << endl;
    auto t = activity.machine->add( new PoseTo("endeffL", ARRAY(pose.pos), ARRAY(pose.rot)));
    activity.machine->waitForActionCompletion(t);
    cout << "aligned" << endl;

    int jointID = -(activity.machine->s->MP.world.getJointByName("l_gripper_joint")->qIndex);
    GroundedAction* action = activity.machine->add( new SetQ("XXX", jointID, {.0}));
    action->tasks(0)->setGains(2000, 0);

    while (true){
      UserInput input = get_input();

      pose = activity.machine->s->MP.world.getShapeByName("endeffL")->X;
      cout << input.joint_pos(0) << endl;
      cout << pose.rot << endl;
      pose.addRelativeRotationDeg(input.joint_pos(0), 1, 0, 0);
      cout << pose.rot << endl;
      t = activity.machine->add( new PoseTo("endeffL", ARRAY(pose.pos), ARRAY(pose.rot)));
      activity.machine->waitForActionCompletion(t);

      cout << "rotated" << endl;

      pose = activity.machine->s->MP.world.getShapeByName("endeffL")->X;
      pose.pos.x += input.joint_pos(1);
      t = activity.machine->add( new PoseTo("endeffL", ARRAY(pose.pos), ARRAY(pose.rot)));
      activity.machine->waitForActionCompletion(t);
    }
  }

  /// move to the given position
  void move_pris(double joint_value) {
    auto pos = activity.machine->s->world.getShapeByName("endeffL")->X.pos;
    auto rot = activity.machine->s->world.getShapeByName("endeffL")->X.rot;

    //activity.machine->add(new AlignEffTo("endeffL", {joint_value, pos.y, pos.z}, {1, 0, 0}));
    auto action = activity.machine->add(
        new MoveEffTo("endeffL", {joint_value, pos.y, pos.z}));
    activity.machine->waitForActionCompletion(action);
  }

  /// rotate to the given position
  void move_joint(double joint_value, char* joint_name="l_wrist_roll_joint") {
    int jointID = -(activity.machine->s->MP.world.getJointByName(joint_name)->qIndex);
    GroundedAction* action = activity.machine->add(
        new SetQ("XXX", jointID, {joint_value}));
    activity.machine->waitForActionCompletion(action);
  }

  // data
  ActionSystem activity;
};


// ============================================================================
int main(int argc, char** argv)
{
  MT::initCmdLine(argc, argv);
  // test_push();
  // idle();
  // idle2();
  // return 0;
  // test_collision();
  // do_the_dance();
  // testActionMachine();

  IcraExperiment experiment;
  experiment.run();

  // set_q();
  // do_the_dance();

  return 0;
}
