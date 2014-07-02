#include <Ors/ors.h>
#include <Ors/RectDist.h>
#include <Gui/opengl.h>

using ors::Shape;

ors::Vector Pa, Pb;

void draw(void*){
  glLoadIdentity();
  glColor(1., 0., 0., .9);
  glDrawDiamond(Pa.x, Pa.y, Pa.z, .1, .1, .1);
  glDrawDiamond(Pb.x, Pb.y, Pb.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(Pa.x, Pa.y, Pa.z);
  glVertex3f(Pb.x, Pb.y, Pb.z);
  glEnd();
  glLoadIdentity();
}

double distance(ors::Shape& A, ors::Shape& B,ors::Vector& Pa, ors::Vector& Pb){
  CHECK(A.type==ors::SSBoxST && B.type==ors::SSBoxST,"");
  CHECK(!A.size[2] && !B.size[2], "can only handle spheres, cylinders & rectangles yet - no boxes");
  ors::Transformation f;
  f.setDifference(A.X, B.X);
  ors::Matrix R = ((f.rot)).getMatrix();
  ors::Vector Asize={A.size[0], A.size[1], 0.};
  ors::Vector Bsize={B.size[0], B.size[1], 0.};
  ors::Vector trans = f.pos; //Asize + f.pos - R*Bsize;
  double d = RectDist(R.p(), trans.p(), (Asize).p(), (Bsize).p(), Pa.p(), Pb.p());
  Pa = A.X * Pa;
  Pb = A.X * Pb;
  return d;
}

void TEST(Distance){
  ors::KinematicWorld W;
  ors::Shape A(W, NoBody), B(W, NoBody);
  A.type = B.type = ors::SSBoxST;
  memmove(A.size, ARR(.1, .2, .0, .001).p, 4*sizeof(double));
  memmove(B.size, ARR(.1, .2, .0, .001).p, 4*sizeof(double));
  A.X.setRandom();
  B.X.setRandom();
  double d=distance(A, B, Pa, Pb);
  cout <<"d=" <<d <<' ' <<(Pa-Pb).length() <<' ' <<Pa <<Pb <<endl;
  ors::Proxy p; p.posA=Pa; p.posB=Pb; p.colorCode=1;
  W.proxies.append( &p );
//  W.gl().add(draw, NULL);
  W.watch(true);
}

int MAIN(int argc, char** argv){

  testDistance();

  return 1;
}
