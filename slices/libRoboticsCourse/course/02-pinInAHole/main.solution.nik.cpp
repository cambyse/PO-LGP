#include <MT/roboticsCourse.h>

void pin_in_a_hole(){
	Simulator S("pin_in_a_hole.ors");
	S.setContactMargin(.02); //this is 2 cm (all units are in meter)
	arr q,y_target,y,J,W,Phi,PhiJ;
	uint n = S.getJointDimension();

	S.getJointAngles(q);
	W.setDiag(1.,n);

	arr y_hole;
	double y_deviation;
	for(uint i=0;i<1000;i++){
		S.kinematicsPos(y_hole,"hole");

		Phi.clear();
		PhiJ.clear();

		//1st task: pin positioned within hole
		S.kinematicsPos(y,"pin");
		S.jacobianPos  (J,"pin");
		y_target = y + .01*(y_hole-y); //exponentially approach the hole
		y_deviation = 1e-2; //(deviation is 1/precision^2)

		Phi .append((y - y_target) / y_deviation);
		PhiJ.append(J / y_deviation);

		//2nd task: collisions
		S.kinematicsContacts(y);
		S.jacobianContacts(J);
		y_target = ARR(0.); //target is zero collision costs
		y_deviation = 1e-2;

		Phi .append((y - y_target) / y_deviation);
		PhiJ.append(J / y_deviation);

		//compute joint updates
		q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
		S.setJointAngles(q);
	}
}

arr FindPosture(){
	Simulator S("pin_in_a_hole.ors");
	S.setContactMargin(.02); //this is 2 cm (all units are in meter)
	arr q,y_target,y,J,W,Phi,PhiJ;
	uint n = S.getJointDimension();

	S.getJointAngles(q);
	W.setDiag(1.,n);
	arr y_hole;
	double y_deviation;
	arr down = ARR(0,0,-1);
	arr q0 = q;//initial pose
	double alfa = .01;//converges also with value 1.0
	for(uint i=0;i<1000;i++){
		S.kinematicsPos(y_hole,"hole");

		Phi.clear();
		PhiJ.clear();

		//1st task: pin positioned within hole
		S.kinematicsPos(y,"pin");
		S.jacobianPos  (J,"pin");
		y_target = (y_hole); //exponentially approach the hole
		y_deviation = 1e-2; //(deviation is 1/precision^2)

		Phi .append((y - y_target) / y_deviation);
		PhiJ.append(J / y_deviation);
		//cout << " error pos " << norm(y - y_target) << endl;

		//2nd task - vector orientation
		S.kinematicsVec(y,"pin");
		S.jacobianVec  (J,"pin");
		y_target = down; //go downward pointing
		Phi .append((y - y_target) / y_deviation);
		PhiJ.append(J / y_deviation);
		//cout << " error vec " << norm(y - y_target) << endl;

		q-= alfa*inverse(~PhiJ*PhiJ+W)*(W*(q-q0) + ~PhiJ*Phi);
		S.setJointAngles(q);
	}
	S.kinematicsContacts(y);
	cout << " collisions at qT " << y << endl;
	S.watch();
	S.setJointAngles(q0);
	return q;
}

void DisplaySineQ(const arr & q){
	Simulator S("pin_in_a_hole.ors");
	S.setContactMargin(.02);
	arr q0;	S.getJointAngles(q0);
	int T = 2000;
	arr Q(T,q.N);
	for(int t = 0; t < T; t++){
		Q[t] = q0 + 0.5*(q-q0)*(1 - cos(3.14*t/T));
	}
	for(int t = 0; t < T; t++){
		S.setJointAngles(Q[t]);

	}
	S.watch();
	S.setJointAngles(q0);
}

void DisplayInterp(const arr & qOld){
	Simulator S("pin_in_a_hole.ors");
	S.setContactMargin(.02);
	uint n = S.getJointDimension();
	arr W;
	W.setDiag(1.,n);
	arr q0,q;
	S.getJointAngles(q0);
	q = q0;
	int T = 2000;

	arr yTarget1,yTarget2,y1,y2,y1temp,y2temp,y,J,Phi,PhiJ;
	S.kinematicsVec(y2,"pin");//initialvalue
	S.kinematicsPos(y1,"pin");
	S.setJointAngles(qOld);
	S.kinematicsVec(yTarget2,"pin");//final values
	S.kinematicsPos(yTarget1,"pin");

	S.setJointAngles(q0);
	for(int t = 0; t < T; t++){
		y1temp = y1 + ((t+1)/(double)T)*(yTarget1-y1);//interpolated targets
		y2temp = y2 + ((t+1)/(double)T)*(yTarget2-y2);

		Phi.clear();
		PhiJ.clear();

		S.kinematicsPos(y,"pin");
		S.jacobianPos(J,"pin");
		double y_deviation = 1e-2; //(deviation is 1/precision^2)
		Phi .append((y - y1temp) / y_deviation);
		PhiJ.append(J / y_deviation);
		S.kinematicsVec(y,"pin");
		S.jacobianVec  (J,"pin");
		Phi .append((y - y2temp) / y_deviation);
		PhiJ.append(J / y_deviation);

		q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
		S.setJointAngles(q);
	}
	//for(int t = 0; t < T; t++){
	//	S.setJointAngles(Q[t]);
	//}
	S.watch();
	S.setJointAngles(q0);
}

void RealCollFree(const arr & qOld){
	Simulator S("pin_in_a_hole.ors");
	S.setContactMargin(.02);
	uint n = S.getJointDimension();
	arr W;
	W.setDiag(1.,n);
	arr q0,q;
	S.getJointAngles(q0);
	q = qOld;
	int T = 1000;
	arr Phi,PhiJ,y,J,y_target;
	arr Q(T,q.N);
	double alpha = 0.04;
	//backward traj to start position
	for(int t = 0; t < T; t++){
		Phi.clear();
		PhiJ.clear();

		//1st task: up or go to home...
		double y_deviation = 1e-2;

		if (t > T/2){//go to initial pos
			y_target = q0;
			y =q;
			J = 2.0*W;
		}
		else{//move up
			S.kinematicsPos(y,"pin");
			S.jacobianPos(J,"pin");
			y_target = ARR(0,-1,1.7);
		}
		Phi .append((y - y_target) / y_deviation);
		PhiJ.append(J / y_deviation);
		cout << " task error " << t << " : " << norm(y - y_target) << endl;

		//2nd task: collisions
		S.kinematicsContacts(y);
		S.jacobianContacts(J);
		y_target = ARR(0.); //target is zero collision costs
		y_deviation = 1e-2;
		Phi .append((y - y_target) / y_deviation);
		PhiJ.append(J / y_deviation);

		q -= alpha*inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
		S.setJointAngles(q);
		Q[T-t-1] = q;
	}
	S.kinematicsContacts(y);
	cout << " collisions at qT " << y << endl;
	S.watch();
	//now f orwrd in time
	for(int t = 0; t < T; t++){
	//	S.watch();
		S.setJointAngles(Q[t]);
	}
	S.watch();
}

int main(int argc,char **argv){
	MT::initCmdLine(argc,argv);

	arr q = FindPosture();//a)
	if(true){//b) the 2 profiles
		DisplaySineQ(q);
		DisplayInterp(q);
	}
	RealCollFree(q);//c)
	//  pin_in_a_hole();

	return 0;
}
