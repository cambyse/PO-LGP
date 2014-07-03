#include <Core/util.h>
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

void distance(double& t, double& u,
              const double& a, const double& b,
              const double& A_dot_B,
              const double& A_dot_T,
              const double& B_dot_T){
  double denom = 1. - (A_dot_B)*(A_dot_B);

  if(denom == 0.) t=0.;
  else{
    t = (A_dot_T - B_dot_T*A_dot_B)/denom;
    ClipToRange(t,-a,a);
  }
  u = t*A_dot_B - B_dot_T;
  ClipToRange(u,-b,b);
  t = u*A_dot_B + A_dot_T;
  ClipToRange(t,-a,a);
}

double distance2(ors::Shape& A, ors::Shape& B,ors::Vector& Pa, ors::Vector& Pb){
  CHECK(A.type==ors::SSBoxST && B.type==ors::SSBoxST,"");
  CHECK(!A.size[1] && !B.size[1] && !A.size[2] && !B.size[2], "can only handle spheres, cylinders & rectangles yet - no boxes");
  ors::Vector tmp;
  ors::Vector a=A.X.rot.getX(tmp);
  ors::Vector b=B.X.rot.getX(tmp);
  ors::Vector T=B.X.pos - A.X.pos;
  double t, u;
  distance(t, u, A.size[0], B.size[0], a*b, a*T, b*T);
  Pa = A.X.pos + t*a;
  Pb = B.X.pos + u*b;
  return (Pa-Pb).length();
}

double distance(ors::Shape& A, ors::Shape& B,ors::Vector& Pa, ors::Vector& Pb){
  CHECK(A.type==ors::SSBoxST && B.type==ors::SSBoxST,"");
  CHECK(!A.size[2] && !B.size[2], "can only handle spheres, cylinders & rectangles yet - no boxes");
  if(!A.size[1] && !B.size[1]){ //SSLines
    return distance2(A, B, Pa, Pb);
  }
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
  memmove(A.size, ARR(1.5, 1.2, .0, .001).p, 4*sizeof(double));
  memmove(B.size, ARR(1.5, 1.2, .0, .001).p, 4*sizeof(double));
  for(uint k=0;k<200;k++){
    A.X.setRandom(); A.X.pos(2) += 2.;
    B.X.setRandom(); B.X.pos(2) += 2.;
    double d=distance(A, B, Pa, Pb);
    double d2=(Pa-Pb).length();
    cout <<"d=" <<d <<' ' <<d2 <<' ' <<Pa <<Pb <<endl;
    if(d>0.) CHECK_ZERO(d-d2, 1e-4, "NOT EQUAL!");
    ors::Proxy p; p.posA=Pa; p.posB=Pb; p.colorCode=1;
    W.proxies.append( &p );
    W.watch(false); MT::wait(.1);
    W.proxies.clear();
  }
}

int MAIN(int argc, char** argv){

  testDistance();

  return 1;
}
