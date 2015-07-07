#include <stdlib.h>
#include <GL/gl.h>

#include <Gui/mesh.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>

ors::Vector p1, p2;
ors::Vector e1, e2;
GJK_point_type pt1, pt2;

void draw(void*){
  glDisable(GL_DEPTH_TEST);

  glColor(1., 0., 0., .9);
  glDrawDiamond(p1.x, p1.y, p1.z, .1, .1, .1);
  glDrawDiamond(p2.x, p2.y, p2.z, .1, .1, .1);
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p2.x, p2.y, p2.z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(p1.x, p1.y, p1.z);
  glVertex3f(p1.x+e1.x, p1.y+e1.y, p1.z+e1.z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(p2.x, p2.y, p2.z);
  glVertex3f(p2.x+e2.x, p2.y+e2.y, p2.z+e2.z);
  glEnd();
  glLoadIdentity();
}

extern bool orsDrawWires;

void TEST(GJK_Jacobians) {
  ors::KinematicWorld W;
  ors::Body base(W), b1(W), B1(W), b2(W), B2(W);
  ors::Joint j1(W, &base, &b1), J1(W, &b1, &B1), j2(W, &base, &b2), J2(W, &b2, &B2);
  ors::Shape s1(W, B1), s2(W, B2);
  j1.type = j2.type = ors::JT_trans3;
  j1.A.addRelativeTranslation(1,1,1);
  j2.A.addRelativeTranslation(-1,-1,1);
  J1.type = J2.type = ors::JT_quatBall;
  s1.type = s2.type = ors::meshST;
  s1.mesh.setRandom();
  s2.mesh.setRandom();

  W.calc_fwdPropagateFrames();
  arr q = W.getJointState();

  //  animateConfiguration(W);
  orsDrawWires=true;
  W.gl().add(draw);

  VectorFunction f = [&W, &s1, &s2](arr& v, arr& J, const arr& x) -> void {
    W.setJointState(x);
    double d2=GJK_sqrDistance(s1.mesh, s2.mesh, s1.X, s2.X, p1, p2, e1, e2, pt1, pt2);
//    if(&J) cout <<"point types= " <<pt1 <<' ' <<pt2 <<endl;
    if(d2<1e-10) LOG(-1) <<"zero distance";
    arr y1, J1, y2, J2;
    W.kinematicsPos(y1, J1, s1.body, s1.body->X.rot/(p1-s1.X.pos));
    W.kinematicsPos(y2, J2, s2.body, s2.body->X.rot/(p2-s2.X.pos));
    v = y1 - y2;
    if(&J){
      J = J1 - J2;
      if((pt1==GJK_vertex && pt2==GJK_face) || (pt1==GJK_face && pt2==GJK_vertex)){
        arr vec, Jv, n = v/length(v);
        J = n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2.body, s2.body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1.body, s1.body->X.rot/(p1-p2));
        J += Jv;
      }
      if((pt1==GJK_vertex && pt2==GJK_edge) || (pt1==GJK_edge && pt2==GJK_vertex)){
        arr vec, Jv, n;
        if(pt1==GJK_vertex) n=ARRAY(e2); else n=ARRAY(e1);
        J = J - n*(~n*J);
        if(pt1==GJK_vertex) W.kinematicsVec(vec, Jv, s2.body, s2.body->X.rot/(p1-p2));
        if(pt2==GJK_vertex) W.kinematicsVec(vec, Jv, s1.body, s1.body->X.rot/(p1-p2));
        J += n*(~n*Jv);
      }
      if(pt1==GJK_edge && pt2==GJK_edge){
        arr vec, Jv, n, a, b;
        n = v/length(v);
        J = n*(~n*J);

        W.kinematicsVec(vec, Jv, s1.body, s1.body->X.rot/e1);
        a=ARRAY(e1);
        b=ARRAY(e2);
        double ab=scalarProduct(a,b);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;

        W.kinematicsVec(vec, Jv, s2.body, s2.body->X.rot/e2);
        a=ARRAY(e2);
        b=ARRAY(e1);
        J += (a-b*ab) * (1./(1.-ab*ab)) * (~v*(b*~b -eye(3,3))) * Jv;
      }
    }
    //      CHECK_ZERO(y.scalar()-d2, 1e-6,"");
  };

  for(uint k=0;k<110;k++){
    rndGauss(q, .3);

    arr y,J;
    //    f(y, J, q);
    checkJacobian(f, q, 1e-4);

    W.gl().update();
  }


}


int MAIN(int argc, char** argv){

  testGJK_Jacobians();

  return 1;
}
