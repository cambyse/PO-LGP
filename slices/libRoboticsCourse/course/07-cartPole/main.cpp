#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/opengl.h>

void drawEnv(void*);
void glDrawCartPole(void *classP);

struct CartPoleState{
  double x,x1,th,th1;
  double tau,x2,the2;
  double c1,c2,Mp,Mc,l,g, dynamicsNoise;
  OpenGL gl;
  CartPoleState(){
    x=0.;
    x1=0.;
    th=.2; //slighly non-upright //MT_PI; //haning down
    th1=0.;

    //init constants
    tau = 1/60.0; //with 1/1000 is better
    Mp = 1;
    Mc = 1;
    l = 1;
    c1 = 1/(Mp+Mc);
    c2 = l*Mp/(Mp+Mc);
    g = 9.8;

    dynamicsNoise = 0;

    gl.add(drawEnv, this);
    gl.add(glDrawCartPole, this);
    gl.camera.setPosition(10., -50., 10.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
    gl.update();
  }

  void step(double u){
    the2 = g*sin(th) + cos(th)*(-c1*u-c2*th1*th1*sin(th));
    the2 /= l*4/3 - c2*cos(th)*cos(th);
    x2 = c1*u + c2*(th1*th1*sin(th) - the2*cos(th));

    x   += tau*x1;
    x1  += tau*x2;
    th  += tau*th1;
    th1 += tau*the2;

    if(dynamicsNoise){
      x1 += dynamicsNoise*rnd.gauss();
      th1 += dynamicsNoise*rnd.gauss();
    }
  }

};

void glDrawCartPole(void *classP){
  CartPoleState *s=(CartPoleState*)classP;
  double GLmatrix[16];
  ors::Transformation f;
  //cart
  f.addRelativeTranslation(s->x,0.,1.);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8,.2,.2);
  glDrawBox(1., .2, .2);
  //pole
  f.addRelativeRotationRad(s->th,0., 1., 0.);
  f.addRelativeTranslation(0., 0., .5);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.2,.2,.2);
  glDrawBox(.1, .1, 1.);
}

void testDraw(){
  CartPoleState s;
  s.gl.watch();
}

void TestMove(){
  CartPoleState s;
  for (uint t=0; t<400000; t++){
    s.gl.text.clear() <<t <<" ; " <<s.x1 << " ; " <<s.th1;
    s.step(0.0);
    s.gl.update();
  }
}

double GetControl(const arr & w, CartPoleState & s){
  return w(0)*s.x + w(1)*s.x1 + w(2)*s.th + w(3)*s.th1 ;
}

double GetCost(CartPoleState & s){
  return s.x*s.x + 0.01*s.th*s.th + s.x1*s.x1 + s.th1*s.th1;
}

void optimize(arr& w, bool noise){
  //Nikolay's optimization heuristic
  CartPoleState s;
  if(noise)  s.dynamicsNoise = 0.01;
  arr wbest;
  w.resize(4);
  w.setZero(0); wbest = w;

  double fbest = 10000;
  for (uint z = 0; z < 100000; z++){
    s.th = 0.1;s.th1 = 0.;	s.x = 0;s.x1 = 0;s.the2 = 0;s.x2 = 0;
    rndGauss(w,100,false);
    //rndUniform()w = w*0.12;//w(3) = w(3)*2;//semms larger
    double cost = 0;
    for (uint t = 0; t < 600; t++){
      if (t >= 550 && t%5 == 0)
	cost+= GetCost(s);//at rest at 0 0
      s.step(GetControl(w,s));
    }
    cost/= 10;
    if (cost < fbest){
      fbest = cost;
      wbest =w;
      cout << " best at " << z << " " << w << " :  " << cost << " x1 " << s.x1 << " th1" << s.th1 << endl;
      s.gl.text.clear() <<z <<  " best: " << fbest;
      s.gl.update();
    }
    if(z%1000 == 0){
      s.gl.text.clear() <<z<<  " best: " << fbest;
      s.gl.update();
    }
  }

  for(uint sd = 0; sd < 2; sd++)
    for (uint z = 0; z < 100000; z++){
      s.th = 0.1;s.th1 = 0.;	s.x = 0;s.x1 = 0;s.the2 = 0;s.x2 = 0;
      w = wbest;
      rndGauss(w,0.6/(1+sd),true);//decreasing std
      double cost = 0;
      for (uint t = 0; t < 600; t++){
	if (t >= 550 && t%5 == 0)
	  cost+= GetCost(s);//at rest at 0 0
	s.step(GetControl(w,s));
      }
      cost/= 10;
      if (cost < fbest){
	fbest = cost;
	wbest =w;
	cout << " best at " << z << " " << w << " :  " << cost << " ; " << s.x1 << " " << s.th1 << endl;
      }
      if(z%1000 == 0){
	s.gl.text.clear() <<z << " " << 0.06/(1+sd) << " best: " << fbest;
	s.gl.update();
      }
    }

  w = wbest;
}

void playController(const arr& w,bool noise){
  CartPoleState s;
  if(noise)  s.dynamicsNoise = 0.01;
  for(uint t=0;;t++){
    s.step(GetControl(w,s));
    s.gl.update();
    s.gl.text.clear() << t;
  }
}


int main(int argc,char **argv){
  //testDraw();
  //TestMove();
  //TestStability(false);//parameter tells whether noise is used or not

  arr w=ARR(1.00000, 2.58375, 52.36463, 15.25927);
  playController(w,true);
  return 0;
}
