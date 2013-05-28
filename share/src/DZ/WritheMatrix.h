#include <MT/ors.h>

void GetWritheMatrix(arr& WM, const arr& rope1, const arr& rope2,int dim1,int dim2);

void GetWritheSegment(double& writhe,const ors::Vector& A,const ors::Vector& B,const ors::Vector& C,const ors::Vector& D);

void GetWritheJacobianSegment(arr& JTscalar,const ors::Vector& p_a,const ors::Vector& p_b,const ors::Vector& p_c,const ors::Vector& p_d, const arr& Jab);
		       
void WritheJacobian(arr& JM, const arr& rope1, const arr& rope2,arr& pointsJ,int dim1,int dim2);

/// Scalar experiments
void GetScalarWrithe(arr& WS, const arr& rope1, const arr& rope2,int dim);

void ScalarJacobian(arr& SJ, const arr& rope1, const arr& rope2,arr& pointsJ,int dim);