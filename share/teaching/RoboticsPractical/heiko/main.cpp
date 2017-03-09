#include <Core/util.h>
#include "../interface/myBaxter.h"
#include <Control/taskControl.h>

#include <ros/ros.h>
#include <RosCom/roscom.h>
#include <ostream>

#include <Core/array.h>
#include <Core/array.tpp>
#include <Core/util.h>
#include <lapack/blaswrap.h>
// =================================================================================================
arr Joint_Nullspace(arr A, arr B){
    //Not working currently
    arr Ua, da, Va, Uc, dc, Vc;
    lapack_full_SVD(Ua, da, Va, A);
    int r = A.d0; //A.rank(A);
    arr C = B * Va.sub(0,-1,r+1, A.d1);
    lapack_full_SVD(Uc, dc, Vc, C);
    int q = 4; //rank(C);
    //arr Y = Va(0,-1,r+1,A.d0)*Vc(0.-1,q+1,n-r);
}


int main(int argc, char** argv){

    mlr::initCmdLine(argc, argv);
    uint samples = mlr::getParameter<uint>("samples", 10);
    uint nsamples = mlr::getParameter<uint>("nsamples", 10);
    {
                MyBaxter baxter;

                auto qItself = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
                CtrlTask posR("endeffR", new TaskMap_Default(posTMT, baxter.getKinematicWorld_Model(), "endeffR", NoVector, "base_footprint"), 1., 1., 1., 1.);
                CtrlTask posL("endeffL", new TaskMap_Default(posTMT, baxter.getKinematicWorld_Model(), "endeffL", NoVector, "base_footprint"), 1., 1., 1., 1.);

                std::ofstream data("data.cv");
                arr q_noise = arr(17);
                arr JL, JR, U, d, V, c, q, q_real, y;
                double alpha = .5;

		for(uint i = 0; i < samples; i++){

                        //Move to random target: q0 + gaussian noise
                        //q = {-0.246781, 0.788001, -0.0799924, -1.16773, -1.18989, 1.94245, -0.822752, 2.24372, 1.81686, -0.625563, 1.10926, 0.330455, 0.877026, 0.48605, -0.680918, 0.244263, 0.0571587};
                        rndGauss(q_noise, 0.25, false);
                        q = baxter.q0() + q_noise;
                        baxter.modifyTarget(qItself, q);
                        baxter.waitConv({qItself});
                        mlr::wait(1.);

                        posL.map.phi(y, JL, baxter.getKinematicWorld_Model());
                        posR.map.phi(y, JR, baxter.getKinematicWorld_Model());
                        //Compute Nullspace using SVD

                        for(uint j = 0; j < nsamples; i++){
                            lapack_full_SVD(U, d, V, JL);

                            //sample random coefficients for nullspace basis vectors
                            c =randn(17, 1);
                            for(uint i=0; i<3; i++) c.elem(i) = 0;
                            d = ~V*c;

                            std::cout << JL << "\n\n";
                            std::cout << V << "\n\n";
                            std::cout << d << "\n\n";
                            std::cout << "Motion: " << JL * d << "\n\n";

                            baxter.modifyTarget(qItself, q + alpha*d);
                            baxter.waitConv({qItself});
                            mlr::wait(2.);

                            //Write out touques and joint angles
                            arr u = baxter.getEfforts();
                            baxter.getKinematicWorld().getJointState(q_real);
                            for(uint l=0; l<10; l++) data << q_real << ";" << u << std::endl;
                        }
		}

		//homing
		baxter.modifyTarget(qItself, baxter.q0());
		baxter.waitConv({qItself});
  }

	cout <<"bye bye" <<endl;
	return 0;
}


