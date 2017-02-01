#include <Kin/kin.h>

void GetWritheMatrix(arr& WM, const arr& rope1, const arr& rope2,int dim1,int dim2);

void GetWritheSegment(double& writhe,const mlr::Vector& A,const mlr::Vector& B,const mlr::Vector& C,const mlr::Vector& D);

void GetWritheJacobianSegment(arr& JTscalar,const mlr::Vector& p_a,const mlr::Vector& p_b,const mlr::Vector& p_c,const mlr::Vector& p_d, const arr& Jab);
		       
void WritheJacobian(arr& JM, const arr& rope1, const arr& rope2,arr& pointsJ,int dim1,int dim2);

/// Scalar experiments
void GetScalarWrithe(arr& WS, const arr& rope1, const arr& rope2,int dim);

void ScalarJacobian(arr& SJ, const arr& rope1, const arr& rope2,arr& pointsJ,int dim);