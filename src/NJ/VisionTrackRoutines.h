#ifndef NJVis

#define NJVis

#include <MT/ors.h>

//from camera parameters to camera center, cf. Zisserman
ors::Vector CameraLocation(const arr & p){
	double X,Y,Z,T;
	arr a(3,3);

	a = p.sub(0,2,1,3);
	X =  determinant(a);
	a = p.sub(0,2,0,2);
	T = -determinant(a);

	for (uint i = 0; i < 3; i++)
		for (uint j = 0; j < 3; j++)if(j == 0)
			a(i,j) = p(i,j);
		else
			a(i,j)= p(i,j+1);
	Y = -determinant(a);

	for (uint i = 0; i < 3; i++)
		for (uint j = 0; j < 3; j++)if(j == 2)
			a(i,j) = p(i,j+1);
		else
			a(i,j)= p(i,j);
	Z = determinant(a);
	ors::Vector ans(X/T,Y/T,Z/T);
	return ans;
}

arr GetCrossPlanes(double x, double y){
	arr planes(2,3);planes.setZero();
	planes(0,2) = x;
	planes(1,2) = y;
	planes(0,0) = -1;
	planes(1,1) = -1;

	return planes;
}

//p4 has xL yL yR yR pixel information, the matrices are loaded from regparams
arr Find3dPoint(const arr & P1,const arr & P2, const arr & p4){
	arr planes1 = GetCrossPlanes(p4(0),p4(1))*P1;
	arr planes2 = GetCrossPlanes(p4(2),p4(3))*P2;

	arr a(4,3),b(4);//[a b] has pi planes on rows
	for(uint i = 0; i< 3; i++){
		a(0,i) = planes1(0,i);
		a(1,i) = planes1(1,i);
		a(2,i) = planes2(0,i);
		a(3,i) = planes2(1,i);
	}
	b(0) = -planes1(0,3);b(1) = -planes1(1,3);
	b(2) = -planes2(0,3);b(3) = -planes2(1,3);

	// cout << " p4 " << p4 << " a " << a << " b " << b << endl;
	arr api;
	arr eye(3,3); eye.setZero(); eye.setDiag(1);
	pseudoInverse(api,a,eye,1e-2);
	arr X = api*b;

	return X;
}

struct SimpleTrack{
	ors::Vector v;//speed
	ors::Vector p;//last position
	double t ;//last time
	int Smooth;//smooth type

	void addSample(ors::Vector pos, double time){
		cout << " timeee " << (time-t) << endl;
		v = (pos-p)/(time-t);
		p = pos;
		t = time;
	}
	ors::Vector predict(double time){
		return p + v*(time-t);
	}
	arr R1,R2,R12,xk,Pk;
	arr C,Ct,y,Phi,Phit;
	arr x,P;//x,p |tt

	void Init(){
		MT::getParameter(Smooth,"Smooth");
		R1 = arr(6,6);R1.setDiag(0.02);
		for(uint i = 0; i < 3; i++){
			R1(i,i+3) = 0.008;//smaller value for better pos
			R1(i+3,i) = 0.008;//cov between speed and pos
		}
		R2 = arr(4,4);R2.setDiag(0.6);
		//arr cov;
		//MT::load(cov, "COV");cov = cov*1.0;//created from measured data
		//R1 = 5.0*cov.sub(0,5,0,5);
		//R2 = 2.0*cov.sub(6,9,6,9);
		R12 = arr(6,4);R12.setZero();
		// R12 = 2.5*cov.sub(0,5,6,9);//R12(0,0)= 0.1;R12(2,1)= 0.1;R12(0,2)= 0.1;R12(2,3)= 0.1;
		Phi = arr(6,6);Phi.setDiag(1.0);//setdiag has setzsero inside
		Pk = arr(6,6); Pk.setDiag(.6);//initial covariance
		xk = arr(6);xk.setZero();xk(0) = 0.2; xk(1) = -0.5; xk(2) = 1;//start guess for x
		t = 0;
	}

	void PrepareObservation(const arr& vision, const arr & Pl, const arr & Pr, double time){
		for(uint i = 0; i < 3; i++)
			Phi(i,i+3)=time-t; //double t = 0.26;//find real time interval, .26 or .14, or 0
		transpose(Phit,Phi);
		t = time;

		C = arr(4,6);C.setZero();
		arr PLt;transpose(PLt,Pl);
		arr PRt;transpose(PRt,Pr);

		arr l(2,3);l.setZero();
		l(0,0) = -1;l(1,1) = -1;
		l(0,2) = vision(0);l(1,2) = vision(1);//left pixels
		arr lt;
		transpose(lt,l);
		arr piL = PLt*lt;    if(vision(0)+vision(1) == 0)         {piL = 0.0;cout << "left 0" << endl; }   //more robust hopefully
		l(0,2) = vision(2);l(1,2) = vision(3);  //right pixels
		transpose(lt,l);
		arr piR = PRt*lt;if(vision(2)+vision(3) == 0)         {piR = 0.0;cout << "right 0" << endl; }
		arr pi(4,4);
		for(uint i = 0; i < 4; i++){
			pi(i,0) = piL(i,0);
			pi(i,1) = piL(i,1);
			pi(i,2) = piR(i,0);
			pi(i,3) = piR(i,1);
		}
		for(uint i = 0; i < 4; i++)
			for(uint j = 0; j < 3; j++)
				C(i,j) = pi(j,i);//careful transposes...
				transpose(Ct,C);
				y = pi.sub(3,3,0,3);y.resize(4);
				y = y*-1.0;
	}

	void addSample(const arr& vision, const arr & Pl, const arr & Pr, double time ){
		PrepareObservation(vision,Pl,Pr,time);

		arr t1 = inverse(R2 + C*Pk*Ct);
		arr K = (Phi*Pk*Ct+R12)*t1;
		arr Kf = Pk*Ct*t1;

		arr oldx = x;
		arr oldP = P;
		arr oldPk = Pk;
		arr oldxk = xk;

		x = xk + Kf*(y-C*xk);
		P = Pk - Kf*C*Pk;//transpose or not ??//arr Pkt; transpose(Pkt,Pk);

		//arr t2;transpose(t2,Phi*Pk*Ct+R12);
		//Pk= Phi*Pk*Phit + R1 - K*t2;error in ppaper equations, no convergence...
		//xk = Phi*xk + K*(y-C*xk);
		xk = Phi*x;//wikipedia notation
		Pk = Phi*P*Phit + R1 ;

		if(oldx.N > 0 && Smooth == 1){ //smooth if data exists, it improves std on positions...
			arr Pi= inverse(oldPk);
			arr Pstar = oldP*Phit*Pi;//cout << "#" << Pstar << "#" << endl;
			x = oldx + Pstar*(x - Phi*oldx);
		}
		if (oldx.N > 0 && Smooth == 2){//RTS filter
			arr Pi= inverse(oldPk);
			arr Phii = inverse(Phi);
			arr Kn = Phii*R1*Pi;
			arr id(6,6);id.setDiag(1.0);
			arr F = Phii*(id - R1*Pi);
			x = F*x + Kn*oldxk;
		}
		if (oldx.N > 0 && Smooth == 3)
			x = 0.5*(oldx +x);//simple average smoother..

		p = ors::Vector(x(0),x(1),x(2));
		v = ors::Vector(x(3),x(4),x(5));
		x = oldx;//to avoid smoothing self effect..
	}
};

struct AverageTrack{
	double Variance;
	ors::Vector v;//speed
	ors::Vector p;//last position
	ors::Vector pdif;//last position difference
	uint Smooth;//smooth type
	arr VSamples;
	double t;


	void Init(){
		MT::getParameter(Smooth,"Smooth");
		VSamples = arr(Smooth,4);//store here vision, average in  vision space
		VSamples.setZero();
	}

	void addSample(const arr& vision, const arr & Pl, const arr & Pr, double time ){
		for(uint i = 1; i < Smooth; i++)
			VSamples[i-1] =  VSamples[i];
		VSamples[Smooth-1] = vision;

		arr avvis(4);avvis.setZero();
		for(uint i = 0; i < Smooth; i++)
			avvis = avvis + VSamples[i]/(double)Smooth;

		arr po = Find3dPoint(Pl,Pr,avvis);
		ors::Vector pnew(po(0),po(1),po(2));
		pdif = (pnew-p);
		v = (pnew-p)/(time-t);//average velocity in m/s, displacement over time
		p = pnew;
		t = time;

		Variance = 0;
		for(uint i = 0; i < Smooth; i++)
			Variance += sumOfSqr(VSamples[i]-avvis)/Smooth;
	}
};

#endif
