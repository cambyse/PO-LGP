#include <Core/util.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>

double
RectDist(double Rab[9], double Tab[3],
double a[2], double b[2], double Pa[3], double Pb[3]);

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

inline void clip(double& x, double r){
  if(x<-r) x=-r; else if(x>r) x=r;
}

double distance_SSPoints(ors::Shape& A, ors::Shape& B,ors::Vector& Pa, ors::Vector& Pb){
  CHECK(A.type==ors::SSBoxST && B.type==ors::SSBoxST,"");
  CHECK(!A.size[0] && !B.size[0] && !A.size[1] && !B.size[1] && !A.size[2] && !B.size[2], "can only handle SSpoints");
  Pa = A.X.pos;
  Pb = B.X.pos;
  ors::Vector c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.size[3]*c/d;
  Pb += B.size[3]*c/d;
  return d-A.size[3]-B.size[3];
}

double distance_SSLinePoint(ors::Shape& A, ors::Shape& B,ors::Vector& Pa, ors::Vector& Pb){
  CHECK(A.type==ors::SSBoxST && B.type==ors::SSBoxST,"");
  CHECK(!B.size[0] && !A.size[1] && !B.size[1] && !A.size[2] && !B.size[2], "can only handle SSLinePoint");
  if(!A.size[0]){ //SSLinePoint
    return distance_SSPoints(A, B, Pa, Pb);
  }
  ors::Vector tmp;
  ors::Vector a=A.X.rot.getX(tmp);
  ors::Vector c=B.X.pos - A.X.pos;
  //get the 'coordinates' along the line segment
  double t = c*a;
  clip(t, A.size[0]);
  //compute closest points
  Pa = A.X.pos + t*a;
  Pb = B.X.pos;
  //distance
  c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.size[3]*c/d;
  Pb += B.size[3]*c/d;
  return d-A.size[3]-B.size[3];
}

double distance_SSLines(ors::Shape& A, ors::Shape& B,ors::Vector& Pa, ors::Vector& Pb){
  CHECK(A.type==ors::SSBoxST && B.type==ors::SSBoxST,"");
  CHECK(!A.size[1] && !B.size[1] && !A.size[2] && !B.size[2], "can only handle SS line segments (cylinders)");
  if(!B.size[0]){ //SSLinePoint
    return distance_SSLinePoint(A, B, Pa, Pb);
  }
  ors::Vector tmp;
  ors::Vector a=A.X.rot.getX(tmp);
  ors::Vector b=B.X.rot.getX(tmp);
  ors::Vector c=B.X.pos - A.X.pos;
  //get the 'coordinates' along the line segments
  double A_dot_B = a*b;
  double A_dot_C = a*c;
  double B_dot_C = b*c;
  double denom = 1. - A_dot_B*A_dot_B;
  double t, u;
  if(denom==0.) t=0.; else t = (A_dot_C - B_dot_C*A_dot_B)/denom;
  clip(t, A.size[0]);
  u = t*A_dot_B - B_dot_C;
  clip(u, B.size[0]);
  t = u*A_dot_B + A_dot_C;
  clip(t, A.size[0]);
  //compute closest points
  Pa = A.X.pos + t*a;
  Pb = B.X.pos + u*b;
  //distance
  c = Pa-Pb;
  double d = c.length();
  //account for radii
  Pa -= A.size[3]*c/d;
  Pb += B.size[3]*c/d;
  return d-A.size[3]-B.size[3];
}

double distance_SSRects(ors::Shape& A, ors::Shape& B,ors::Vector& Pa, ors::Vector& Pb){
  CHECK(A.type==ors::SSBoxST && B.type==ors::SSBoxST,"");
  CHECK(!A.size[2] && !B.size[2], "can only handle spheres, cylinders & rectangles yet - no boxes");
  if(!A.size[1] && !B.size[1]){ //SSLines
    return distance_SSLines(A, B, Pa, Pb);
  }
  ors::Transformation f;
  f.setDifference(A.X, B.X);
  ors::Matrix R = ((f.rot)).getMatrix();
  ors::Vector Asize={A.size[0], A.size[1], 0.};
  ors::Vector Bsize={B.size[0], B.size[1], 0.};
  ors::Vector trans = Asize + f.pos - R*Bsize;
  double dist = RectDist(R.p(), trans.p(), (2.*Asize).p(), (2.*Bsize).p(), Pa.p(), Pb.p());
  Pa = A.X * (Pa-Asize);
  Pb = A.X * (Pb-Bsize);
  //distance
  ors::Vector c = Pa-Pb;
  double d = c.length();
  if(dist>0.) CHECK_ZERO(dist-d, 1e-4, "NOT EQUAL!");
  if(dist==0.) d *= -1.; //if the rects penetrate already, measure the penetration as negative!
  //account for radii
  Pa -= A.size[3]*c/d;
  Pb += B.size[3]*c/d;
  return d-A.size[3]-B.size[3];
}

void TEST(Distance){
  ors::KinematicWorld W;
  ors::Shape A(W, NoBody), B(W, NoBody);
  A.type = B.type = ors::SSBoxST;
  memmove(A.size, ARR(.0, .0, .0, .1).p, 4*sizeof(double));
  memmove(B.size, ARR(.0, .0, .0, .1).p, 4*sizeof(double));
  for(uint k=0;k<200;k++){
    A.X.setRandom(); A.X.pos(2) += 2.;
    B.X.setRandom(); B.X.pos(2) += 2.;
    double d=distance_SSRects(A, B, Pa, Pb);
    double d2=(Pa-Pb).length();
    cout <<"d=" <<d <<' ' <<d2 <<' ' <<Pa <<Pb <<endl;
    if(d>0.) CHECK_ZERO(d-d2, 1e-4, "NOT EQUAL!");
    ors::Proxy p; p.posA=Pa; p.posB=Pb; p.colorCode=1;
    W.proxies.append( &p );
    W.watch(true); MT::wait(.1);
    W.proxies.clear();
  }
}

int MAIN(int argc, char** argv){

  testDistance();

  return 1;
}
