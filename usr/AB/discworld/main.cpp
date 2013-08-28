#include <string>

#include <Ors/ors_physx.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>

#define HEIGHT      (.1)
#define HALF_HEIGHT ((HEIGHT) / 2)
#define GOAL_HEIGHT (2.5*(HEIGHT))
#define RADIUS      (.3)

void createOrs(ors::Graph& ors, OpenGL& gl) {
  MT::Array<ors::Vector> bodies;

  ors::Body *b;
  ors::Shape *s;

  ors.clear();
  
  b = new ors::Body(ors);
  b->name << "agent";
  b->type = ors::kinematicBT;
  b->X.pos.setZero();
  b->X.pos.z = HALF_HEIGHT;
  b->X.rot.setZero();
  bodies.append(b->X.pos);

  s = new ors::Shape(ors, b);
  s->type = ors::cylinderST;
  s->size[2] = HEIGHT;
  s->size[3] = RADIUS;
  s->color[0] = 1;
  s->color[1] = 0;
  s->color[2] = 0;

  bool exists;
  int nbodies = 3;
  for(uint k = 0; k < nbodies; k++) {
    b = new ors::Body(ors);
    b->name << "disc_" << k;
    b->type = ors::dynamicBT;
    b->X.rot.setZero();
    exists = true;
    while(exists) {
      b->X.pos.setRandom(3);
      b->X.pos.z = HALF_HEIGHT;
      exists = false;
      for(int i = 0; i < bodies.N; i++) {
        if((bodies(i) - b->X.pos).length() <= 2*RADIUS) {
          exists = true;
          break;
        }
      }
      bodies.append(b->X.pos);
    }
    b->mass = 100000*(k+1);

    s = new ors::Shape(ors, b);
    s->type = ors::cylinderST;
    s->size[2] = HEIGHT;
    s->size[3] = RADIUS;
    s->color[0] = float(nbodies - k) / nbodies;
    s->color[1] = float(nbodies - k) / nbodies;
    s->color[2] = float(nbodies - k) / nbodies;
  }

  b = new ors::Body(ors);
  b->name << "goal";
  b->type = ors::kinematicBT;
  b->X.pos.setZero();
  b->X.pos.z = GOAL_HEIGHT;
  b->X.rot.setZero();

  s = new ors::Shape(ors, b);
  s->type = ors::sphereST;
  s->size[2] = 2*HEIGHT;
  s->size[3] = .1;
  s->color[0] = 0;
  s->color[1] = 0;
  s->color[2] = 1;

  ors.calcShapeFramesFromBodies();
  cout << ors << endl;
  
  gl.add(glStandardScene,NULL);
  gl.add(ors::glDrawGraph, &ors);
  gl.setClearColors(1., 1., 1., 1.);
  gl.camera.setPosition(0., 0., 50.);
  gl.camera.focus(0, 0, 0);
  gl.update();
}

void new_goal(ors::Body *goal) {
  goal->X.pos.setRandom(4);
  goal->X.pos.z = GOAL_HEIGHT;
}

bool move(ors::Body *agent, ors::Body *goal) {
  float speed = .03;

  ors::Vector dir = goal->X.pos - agent->X.pos;
  dir.z = 0;

  bool arrived = false;
  if(dir.length() <= speed)
    arrived = true;
  dir.setLength(speed);
  agent->X.pos += dir;

  return arrived;
}

ostream& operator<<(ostream &os, const ors::Graph *ors) {
  ors::Body *b;
  for(int i = 0; i < ors->bodies.N; i++) {
    b = ors->bodies(i);
    if(b->name != "goal")
      os << b->X.pos.x << " " << b->X.pos.y << " ";
  }
  os << endl;
  return os;
}

void rec_discworld(PhysXInterface &physx, OpenGL &gl, String fname) {
  int T = 200;

  gl.watch();

  ors::Body *agent = physx.G->getBodyByName("agent");
  ors::Body *goal = physx.G->getBodyByName("goal");

  ofstream fout;
  fout.open(fname);
  fout << 2*(physx.G->bodies.N - 1) << " " << T << " " << 2 << endl;
  fout << physx.G;

  new_goal(goal);
  for(uint t = 0; t < T; t++) {
    if(move(agent, goal))
      new_goal(goal);

    cout <<"\r t = " << t << flush;
    physx.step();
    gl.update();

    fout << physx.G;
  }

  fout.close();

  gl.watch();
}

void read_file(String fname, arr &s, int *A) {
  int N, T;
  ifstream fin;
  fin.open(fname);

  fin >> N >> T >> *A;
  s.resize(T, N);

  // optimizable
  for(int t = 0; t < T; t++)
    for(int n = 0; n < N; n++)
      fin >> s(t, n);

  fin.close();
}

void init_ors(ors::Graph &ors, arr &s, int A) {
  ors::Body *b;
  ors::Shape *sh;

  ors.clear();
  int N = s.d1;
  
  String name;
  for(int i = 0; i < N/2; i++) {
    name.clear() << i;
    
    b = new ors::Body(ors);
    b->type = ors::kinematicBT;
    b->name = name;
    b->X.pos.setZero();
    b->X.pos.x = s(0, 2*i);
    b->X.pos.y = s(0, 2*i+1);
    b->X.pos.z = HALF_HEIGHT;
    b->X.rot.setZero();

    sh = new ors::Shape(ors, b);
    sh->type = ors::cylinderST;
    sh->size[2] = HEIGHT;
    sh->size[3] = RADIUS;
    if(i == 0) {
      sh->color[0] = 1;
      sh->color[1] = 0;
      sh->color[2] = 0;
    }
    else
      sh->color[0] = sh->color[1] = sh->color[2] = 1.0/3.0;
  }
  ors.calcShapeFramesFromBodies();
}

void step(ors::Graph &ors, arr &s, int t) {
  ors::Body *b;
  String name;
  for(int i = 0; i < ors.bodies.N; i++) {
    name.clear() << i;

    b = ors.getBodyByName(name);
    b->X.pos.x = s(t, 2*i);
    b->X.pos.y = s(t, 2*i+1);
  }
  ors.calcShapeFramesFromBodies();
}

void play_discworld(ors::Graph &ors, OpenGL &gl, arr &s, int A) {
  int T = s.d0;
  int N = s.d1;

  cout << "T = " << T << endl;
  cout << "N = " << N << endl;

  init_ors(ors, s, A);

  gl.add(glStandardScene,NULL);
  gl.add(ors::glDrawGraph, &ors);
  gl.setClearColors(1., 1., 1., 1.);
  gl.camera.setPosition(0., 0., 50.);
  gl.camera.focus(0, 0, 0);
  gl.update();

  gl.watch();

  // SET
  int lwin = 10; 

  int fwin, twin;
  arr win, tr, te, Phy, PhyT, temp;
  MT::Array<arr*> blocks;
  arr plot = zeros(N, T);

  for(int t = 0; t < T; t++) {
    cout << "\r t = " << t << flush;
    step(ors, s, t);
    gl.update();

    if(T-t > 10) {
      fwin = t;
      twin = fwin + lwin - 1;
      win = s.sub(fwin, twin, 0, -1)();

      tr = win.sub(0, -1, 0, A-1)();
      te = win.sub(0, -1, A, -1)();

      blocks.clear();
      temp = ones(lwin, 1);
      blocks.append(&temp);
      blocks.append(&tr);
      Phy = catCol(blocks);

      PhyT = ~Phy;

      arr W = inverse(PhyT*Phy)*PhyT*te;
      arr y = Phy*W;

      arr diff = y-te;

      diff = ~diff;
      cout << "diff.d0 = " << diff.d0 << endl;
      cout << "diff.d1 = " << diff.d1 << endl;
      for(int n = 0; n < N-A; n++)
        plot(n, t) = norm(diff[n]);
      gnuplot(plot[5]);
      // TODO fix plots
    }
    usleep(100000);
  }
  gl.watch();
}

int main(int argc, char** argv) {
  MT::rnd.clockSeed();
  MT::rnd.seed(4);

  ors::Graph ors;
  OpenGL gl;
  createOrs(ors, gl);
  
  PhysXInterface physx;
  physx.G = &ors;
  physx.create();

  String fname("discworld");

  rec_discworld(physx, gl, fname);

  ors.clear();
  gl.clear();

  arr s;
  int A;
  read_file(fname, s, &A);

  play_discworld(ors, gl, s, A);

  /*
  int lwin = 10;
  int dwin = 5;
  train(s, A, lwin, dwin);

  gl.watch();
  */

  return 0;
}

