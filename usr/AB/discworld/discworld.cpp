#include <stdlib.h>
#include "discworld.h"

#include <unistd.h>

float const DiscWorld::HEIGHT       = .1;
float const DiscWorld::GOAL_HEIGHT  = .25;
float const DiscWorld::RADIUS       = .2;

DiscWorld::DiscWorld(): ors(NULL), gl(NULL), physx(NULL), N(0), T(200), G(0) { }

DiscWorld::~DiscWorld() { }

void DiscWorld::clear() {
  bodies.clear();
  names.clear();
  goals.clear();
  kf.clearState();

  N = 0;
  G = 0;
}

void DiscWorld::clearGui() {
  delete ors;
  delete gl;
  delete physx;

  ors = new ors::KinematicWorld;
  gl = new OpenGL;
  physx = new PhysXInterface;
}

void DiscWorld::initBodies() {
  CHECK(N > 1, "Insert at least 2 bodies into the disc world.");

  clearGui();

  ors::Body *b;
  ors::Shape *s;
  for(int i = 0; i < N; i++) {
    b = new ors::Body(*ors);
    if(i == 0) {
      b->name << "A";
      b->type = ors::kinematicBT;
    }
    else {
      b->name << i;
      b->type = ors::dynamicBT;
    }
    b->X.pos.setZero();
    b->X.rot.setZero();
    b->X.pos = bodies(i);
    b->X.pos.z = HEIGHT/2;
    names.append(new MT::String(b->name));

    s = new ors::Shape(*ors, b);
    s->type = ors::cylinderST;
    s->size[2] = HEIGHT;
    s->size[3] = RADIUS;
    if(i == 0) {
      s->color[0] = 1;
      s->color[1] = s->color[2] = 0;
    }
    else
      s->color[0] = s->color[1] = s->color[2] = .5;
  }

  // goal body
  b = new ors::Body(*ors);
  b->name << "G";
  b->type = ors::kinematicBT;
  b->X.pos.setZero();
  b->X.pos.z = GOAL_HEIGHT;
  b->X.rot.setZero();
  s = new ors::Shape(*ors, b);
  s->type = ors::sphereST;
  s->size[2] = 2*HEIGHT;
  s->size[3] = .1;
  s->color[0] = 0;
  s->color[1] = 0;
  s->color[2] = 1;

  ors->calcShapeFramesFromBodies();
  //bindOrsToPhysX(ors, gl, physx);
  
  gl->clear();
  gl->add(glStandardScene, NULL);
  gl->add(ors::glDrawGraph, ors);
  gl->setClearColors(1., 1., 1., 1.);
  gl->camera.setPosition(0., 0., 50.);
  gl->camera.focus(0, 0, 0);
  gl->update();
  
  physx->G = ors;
  physx->create();
}

void DiscWorld::play() {
  initBodies();
  ors::Body *agent = ors->getBodyByName("A");
  ors::Body *goal = ors->getBodyByName("G");

  kf.clearFrames();

  if(mode)
    cout << "Running until t == " << T << endl;
  else
    cout << "Running until g == " << G << endl;

  gl->watch();
  setGoal(goal, 0);
  for(int g = 0, t = 0; (mode && t < T) || (!mode && g < G); t++) {
    cout << "\r t = " << t << ", g = " << g << flush;

    recStep(t);
    if(move(agent, goal)) {
      g++;
      setGoal(goal, g);
    }

    physx->step();
    gl->update();
  }
  cout << endl << endl;
  gl->watch();
}

void DiscWorld::resetBodies() {
  clearGui();

  ors::Body *b;
  ors::Shape *s;
  for(int i = 0; i < N; i++) {
    b = new ors::Body(*ors);
    b->name << *names(i);
    b->type = ors::kinematicBT;
    b->X.rot.setZero();
    names.append(new MT::String(b->name));

    s = new ors::Shape(*ors, b);
    s->type = ors::cylinderST;
    s->size[2] = HEIGHT;
    s->size[3] = RADIUS;
    if(i == 0) {
      s->color[0] = 1;
      s->color[1] = s->color[2] = 0;
    }
    else
      s->color[0] = s->color[1] = s->color[2] = .5;
  }

  playStep(0);

  ors->calcShapeFramesFromBodies();
  //bindOrsToPhysX(ors, gl, physx);
  
  gl->clear();
  gl->add(glStandardScene, NULL);
  gl->add(ors::glDrawGraph, ors);
  gl->setClearColors(1., 1., 1., 1.);
  gl->camera.setPosition(0., 0., 50.);
  gl->camera.focus(0, 0, 0);
  gl->update();
  
  physx->G = ors;
  physx->create();
}

void DiscWorld::replay() {
  resetBodies();

  gl->watch();

  kf.setAgent(0);
  kf.run();

  MT::String cmd;
  int T = kf.getNFrames();
  cout << "Replay until t == " << T << endl;
  for(uint t = 0; t < T; t++) {
    cout << "\r t = " << t << flush;

    playStep(t);
    gl->update();

    if(t <= lwin)
      continue;

    arr TEMP;
    // TODO try to re-move the plot outside
    MT::String basename("z.pltX_b");
    MT::String fname;
    MT::IOraw = true;
    MT::arrayBrackets = "\0\0";
    for(int b = 0; b < kf.getNBodies(); b++) {
      fname.clear() << basename << b;
      TEMP = kf.getErr(b);
      TEMP.subRange(t-lwin, T-lwin-1).setZero();
      MT::save(TEMP, fname);
    }
    MT::arrayBrackets = "[]";
    MT::IOraw = false;

    cmd.clear() << "set title ''; set key off; set multiplot layout " << N << ", 1; ";
    for(int b = 0; b < kf.getNBodies(); b++)
      cmd << "plot '" << basename << b << "' us 1 lt rgb \"red\", '' us 2 lt rgb \"blue\"; ";
    cmd << "unset multiplot" << endl;
    gnuplot(cmd);
    usleep(40000);
  }
  cout << endl << endl;
  gl->watch();
}

void DiscWorld::setGoal(ors::Body *goal, int g) {
  if(g < G)
    goal->X.pos = goals(g);
  else
    goal->X.pos.setRandom(4);
  goal->X.pos.z = GOAL_HEIGHT;
}

bool DiscWorld::move(ors::Body *agent, ors::Body *goal) {
  ors::Vector dir = goal->X.pos - agent->X.pos;
  dir.z = 0;

  bool arrived = (dir.length() <= speed);

  dir.setLength(speed);
  agent->X.pos += dir;

  return arrived;
}

void DiscWorld::recStep(int t) {
  arr st;
  ors::Body *b;
  for(int n = 0; n < ors->bodies.N; n++) {
    b = ors->bodies(n);
    if(b->name == "G") // this is fine because the goal is the last body
      continue;
    st.append(b->X.pos.x);
    st.append(b->X.pos.y);
  }
  kf.addState(st);
}

void DiscWorld::playStep(int t) {
  ors::Body *b;
  arr st = kf.getState(t);
  for(int n = 0; n < ors->bodies.N; n++) {
    b = ors->getBodyByName(*names(n));
    b->X.pos.x = st(2*n);
    b->X.pos.y = st(2*n+1);
    b->X.pos.z = HEIGHT/2;
  }
  ors->calcShapeFramesFromBodies();
}

void DiscWorld::setTMode(int t) { mode = true; if(t > 0) T = t; }
void DiscWorld::setGMode() { mode = false; }

void DiscWorld::addBody(int n) {
  ors::Vector p;
  for(int i = 0; i < n; ) {
    p.setRandom(3);
    p.z = HEIGHT/2;
    if(!addBody(p))
      continue;
    i++;
  }
}

bool DiscWorld::addBody(float x, float y) {
  return addBody(ors::Vector(x, y, 0));
}

bool DiscWorld::addBody(const ors::Vector &p) {
  for(int i = 0; i < bodies.N; i++)
    if((bodies(i) - p).length() <= 2*RADIUS)
      return false;

  N++;
  bodies.append(p);
  kf.addBody(2);
  return true;
}

void DiscWorld::addGoal(int n) {
  ors::Vector p;
  for(int i = 0; i < n; i++) {
    p.setRandom(3);
    addGoal(p);
  }
}

void DiscWorld::addGoal(float x, float y) {
  addGoal(ors::Vector(x, y, 0));
}

void DiscWorld::addGoal(const ors::Vector &p) {
  G++;
  goals.append(p);
}

void DiscWorld::setSpeed(float s) { speed = s; }
void DiscWorld::setLWin(int l) { lwin = l; kf.setLWin(l); }

