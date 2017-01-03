#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>
#include <Ors/ors_swift.h>


void addDoor(mlr::KinematicWorld &G, double door_h=2.8, double door_w = 1.2, double handle_h = 0.03, double handle_w = 0.4, double handle_y = 0.05,double handle_z=1.) {

  mlr::Body *frame = new mlr::Body(G);
  frame->X.pos = mlr::Vector(1.2, 1.2, door_h/2.);
  frame->type = mlr::BodyType::BT_static;
  mlr::Shape *frame_shape = new mlr::Shape(G,*frame);
  frame_shape->type = mlr::ShapeType::ST_cylinder;
  arr size = ARRAY(0.,0.,door_h,0.0085);
  memmove(frame_shape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(0.,0.,0.);
  memmove(frame_shape->color, color.p, 3*sizeof(double));

  mlr::Body *door = new mlr::Body(G);
  door->type = mlr::BodyType::BT_static;
  door->name = "door";
  mlr::Shape *door_shape = new mlr::Shape(G,*door);
  door_shape->type = mlr::ShapeType::ST_box;
  size = ARRAY(.038, door_w, door_h, 0.);
  memmove(door_shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.5,0.5,0.);
  memmove(door_shape->color, color.p, 3*sizeof(double));

  mlr::Body *handle = new mlr::Body(G);
  handle->type = mlr::BodyType::BT_static;
  handle->name = "handle";
  mlr::Shape *handle_shape = new mlr::Shape(G,*handle);
  handle_shape->type = mlr::ShapeType::ST_box;
  size = ARRAY(.014, handle_w, handle_h, 0.);
  memmove(handle_shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,0.,0.);
  memmove(handle_shape->color, color.p, 3*sizeof(double));

  mlr::Joint *frame_door = new mlr::Joint(G,frame,door);
  frame_door->name = "frame_door";
  frame_door->A.pos = ARR(0, 0, .0);
  frame_door->B.pos = ARR(0, -door_w/2.-0.0085, .0);
  frame_door->type = mlr::JointType::JT_hingeZ;

  mlr::Joint *door_handle = new mlr::Joint(G,door,handle);
  door_handle->name = "door_handle";
  door_handle->A.pos = ARR(-0.07, -door_w/2.+handle_y, -door_h/2.+handle_z);
  door_handle->B.pos = ARR(0, handle_w/2. -0.015, .0);
  door_handle->type = mlr::JointType::JT_hingeX;
  door_handle->limits = ARR(-0.5,0.5);

  mlr::Shape *cp1 = new mlr::Shape(G,*handle);
  cp1->name = "cp1";
  cp1->rel.pos = mlr::Vector(0,0.05, handle_h/2.);
  cp1->type = mlr::ShapeType::ST_sphere;
  size=ARRAY(.1, .1, .1, .01);
  memmove(cp1->size, size.p, 4*sizeof(double));
  color = ARRAY(1.,0.,0.);
  memmove(cp1->color, color.p, 3*sizeof(double));

  mlr::Shape *cp2 = new mlr::Shape(G,*handle);
  cp2->name = "cp2";
  cp2->rel.pos = mlr::Vector(0,-0.02, -handle_h/2.);
  cp2->type = mlr::ShapeType::ST_sphere;
  size=ARRAY(.1, .1, .1, .01);
  memmove(cp2->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(cp2->color, color.p, 3*sizeof(double));

  G.calc_fwdPropagateFrames();
}

void TEST(Door){
  for (;;) {
    mlr::KinematicWorld G("model.kvg");
    G.meldFixedJoints();
    G.removeUselessBodies();
    makeConvexHulls(G.shapes);
    G.watch(true);
    /// sample door
    double door_h = max(ARR(2.+randn(1).first()*0.4,1.));
    double door_w = max(ARR(.85+randn(1).first()*0.2,0.3));
    double handle_h = max(ARR(0.03+randn(1).first()*0.01,0.01));
    double handle_w = max(ARR(0.2+randn(1).first()*0.02,0.05));
    double handle_y = 0.05+randn(1).first()*0.01;
    double handle_z = 1.+randn(1).first()*0.1;
    addDoor(G, door_h, door_w,handle_h,handle_w,handle_y,handle_z);

    G.watch(false);  G.gl().resize(800,800);
    arr q;
    G.getJointState(q);
    G.watch(false);
    MotionProblem MP(G);
    cout <<"joint dimensionality=" <<q.N <<endl;

    //-- setup the motion problem
    Task *t;
    t = MP.addTask("transitions", new TaskMap_Transition(G),OT_sumOfSqr);
    t->map.order=2; //make this an acceleration task!
    t->setCostSpecs(0, MP.T, {0.}, 1e-1);

    double contactT = MP.T/2.;
    // position task maps
    t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeffL", NoVector, "cp1",NoVector),OT_sumOfSqr);
    t->setCostSpecs(contactT-10., contactT, {0.}, 1e2);

    t = MP.addTask("handle_joint", new TaskMap_qItself(G.getJointByName("door_handle")->qIndex, G.getJointStateDimension()),OT_sumOfSqr);
    t->setCostSpecs(contactT+10., contactT+10., {-.3}, 1e3);

    t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("frame_door")->qIndex, G.getJointStateDimension()),OT_sumOfSqr);
    t->setCostSpecs(MP.T-1, MP.T, {-.7}, 1e2);

    // constraints
    t = MP.addTask("contact", new PointEqualityConstraint(G, "endeffC1",NoVector, "cp1",NoVector),OT_eq);
    t->setCostSpecs(contactT, MP.T, {0.}, 1.);
    t = MP.addTask("contact", new PointEqualityConstraint(G, "endeffC2",NoVector, "cp2",NoVector),OT_eq);
    t->setCostSpecs(contactT, MP.T, {0.}, 1.);

    t = MP.addTask("door_fixation", new qItselfConstraint(G.getJointByName("frame_door")->qIndex, G.getJointStateDimension()),OT_eq);
    t->setCostSpecs(0.,contactT+10, {0.}, 1.);

    t = MP.addTask("handle_fixation", new qItselfConstraint(G.getJointByName("door_handle")->qIndex, G.getJointStateDimension()),OT_eq);
    t->setCostSpecs(0.,contactT, {0.}, 1.);

    ShapeL except = G.getBodyByName("l_wrist_roll_link")->shapes;
    t = MP.addTask("collision", new ProxyConstraint(allExceptListedPTMT, shapesToShapeIndices(except), 0.05),OT_ineq);
    t->setCostSpecs(0., MP.T, {0.}, 1.);

    t = MP.addTask("qLimits", new LimitsConstraint(),OT_ineq);
    t->setCostSpecs(0., MP.T, {0.}, 1.);

    //-- create the Optimization problem (of type kOrderMarkov)
    arr x = MP.getInitialization();
    x.reshape(MP.T,G.getJointStateDimension());
    arr lambda = zeros(x.d0,2);

    optConstrained(x, NoArr, Convert(MP), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

    MP.costReport();
    displayTrajectory(x, 1, G, "planned trajectory");
    displayTrajectory(x, 1, G, "planned trajectory");
  }
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  testDoor();

  return 0;
}


