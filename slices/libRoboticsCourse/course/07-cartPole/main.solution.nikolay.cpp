#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/opengl.h>

void drawEnv(void*);
void glDrawCartPole(void *classP);

struct CartPoleState{
	double x,v,theta,omega;
	double tau,x2,the2;
	double c1,c2,Mp,Mc,l,g, dynamicsNoise;
	OpenGL gl;
	CartPoleState(){
		x=0.;
		v=0.;
		theta=.05; //slighly non-upright //MT_PI; //haning down
		omega=0.;
		//init constants
		tau = 1/60.0;
		Mp = 1;//1
		Mc =1;
		l = 1;//1
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
		the2 = g*sin(theta) + cos(theta)*(-c1*u-c2*omega*omega*sin(theta));
		the2 /= l*4/3 - c2*cos(theta)*cos(theta);
		x2 = c1*u + c2*(omega*omega*sin(theta) - the2*cos(theta));
		v  += tau*x2;
		omega += tau*the2;
		x += tau*v;
		theta += tau*omega;
		//keep angles in range....
		if (theta > MT_PI)
			theta -= 2*MT_PI;
		if (theta <= -MT_PI)
			theta += 2*MT_PI;

		if(dynamicsNoise){
			x += dynamicsNoise*rnd.gauss();
			theta += dynamicsNoise*rnd.gauss();
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
	f.addRelativeRotationRad(s->theta,0., -1., 0.);
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
	for (uint t = 0; t < 40000; t++){
		s.gl.text.clr() << t << " ; " << s.v << " ; " << s.omega;
		s.step(0.0);
		s.gl.update();
	}
}

double GetControl(const arr & w, CartPoleState & s){
	if(w.N ==4){
		double v = w(0)*s.x + w(1)*s.v + w(2)*s.theta + w(3)*s.omega ;
		return v;
		/*if(v< 0)
			return -20;
		else
			return 20;*/
	}
}

double GetCost(CartPoleState & s){
	return fabs(s.x) + 0.01*fabs(s.theta) + fabs(s.v) + fabs(s.omega);
}

void TestStability(bool bNoise){
	CartPoleState s;
	if(bNoise)
		s.dynamicsNoise = 0.01;
	double fbest = 10000;
	arr wbest;
	arr w(4);w.setZero(0);wbest = w;
	if(true)
		for (uint z = 0; z < 100000; z++){
			s.theta = 0.1;s.omega = 0.;	s.x = 0;s.v = 0;s.the2 = 0;s.x2 = 0;
			rndGauss(w,100,false);
			//rndUniform()w = w*0.12;//w(3) = w(3)*2;//semms larger
			double cost = 0;
			for (uint t = 0; t < 600; t++){
				if (t >= 550 && t%10 == 0)
					cost+= GetCost(s);//at rest at 0 0
				s.step(GetControl(w,s));
			}
			cost/= 5;
			if (cost < fbest){
				fbest = cost;
				wbest =w;
				cout << " best at " << z << " " << w << " :  " << cost << " v " << s.v << " omega" << s.omega << endl;
				s.gl.text.clr() <<z <<  " best: " << fbest;
				s.gl.update();
			}
			if(z%1000 == 0){
				s.gl.text.clr() <<z<<  " best: " << fbest;
				s.gl.update();
			}
		}

	for(uint sd = 0; sd < 3; sd++)
		for (uint z = 0; z < 50000; z++){
			s.theta = 0.1;s.omega = 0.;	s.x = 0;s.v = 0;s.the2 = 0;s.x2 = 0;
			w = wbest;
			rndGauss(w,0.6/(1+sd),true);//decreasing std
			double cost = 0;
			for (uint t = 0; t < 600; t++){
				if (t >= 550 && t%10 == 0)
					cost+= GetCost(s);//at rest at 0 0
				s.step(GetControl(w,s));
			}
			cost/= 5;
			if (cost < fbest){
				fbest = cost;
				wbest =w;
				cout << " best at " << z << " " << w << " :  " << cost << " ; " << s.v << " " << s.omega << endl;
			}
			if(z%1000 == 0){
				s.gl.text.clr() <<z << " " << 0.06/(1+sd) << " best: " << fbest;
				s.gl.update();
			}
		}

	w = wbest;
	while(true)
	{
		s.theta = 0.1;s.omega = 0.;	s.x = 0;s.v = 0;s.the2 = 0;s.x2 = 0;
		s.gl.watch();
		for (uint t = 0; t < 2000; t++){
			s.step(GetControl(w,s));
			s.gl.update();
			s.gl.text.clr() << t;
		}
	}
}

int main(int argc,char **argv){
	//testDraw();
	//TestMove();
	TestStability(true);//parameter tells whether noise is used or not
	return 0;
}
