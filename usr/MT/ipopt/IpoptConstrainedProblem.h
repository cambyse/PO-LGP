#include <Optim/optimization.h>
#include <coin/IpTNLP.hpp>

struct IpoptConstrainedProblem : TNLP{
  uint n;
  arr x0;
  ConstrainedProblem P;

  //-- buffers to avoid recomputing gradients
  arr x;          ///< point where P was last evaluated
  arr phi_x, J_x, H_x; ///< everything else at x
  TermTypeA tt_x; ///< everything else at x


  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style){

    n = this->n;

    P(phi_x, J_x, H_x, tt_x, x0);

    m=0;
    for(auto t:tt_x) if(t==ineqTT || t=eqTT) m++;
    nnz_jac_g = m*n;
    nnz_h_lag = n*n;
    index_style = TNLP::C_STYLE;
  }

  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u){
    for (Index i=0; i<n; i++)  x_l[i] = -10.0;
    for (Index i=0; i<n; i++)  x_u[i] = +10.0;

    uint ng=0;
    for(auto t:tt_x){
      if(t==ineqTT){ g_l[ng]=-1e10; g_u[ng]=0.; }
      if(t==eqTT){ g_l[ng]=g_u[ng]=0.; }
      ng++;
    }
    CHECK(ng==m,"");

    return true;
  }

  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda){

    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    for(uint i=0;i<n;i++) x[i] = x0(i);
  }

  virtual bool eval_f(Index n, const Number* _x, bool new_x, Number& obj_value){
    if(new_x){
      x.setCarray(_x, n);
      P(phi_x, J_x, H_x, tt_x, x);
    }

    double f=0.;
    for(uint i=0;i<phi_x.N;i++){
      if(            tt_x(i)==fTT                    ) f += phi_x(i);                // direct cost term
      if(            tt_x(i)==sumOfSqrTT             ) f += mlr::sqr(phi_x(i));       // sumOfSqr term
    }
    obj_value = f;
  }

  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f){
    if(new_x){
      x.setCarray(_x, n);
      P(phi_x, J_x, H_x, tt_x, x);
    }

    arr coeff=zeros(phi_x.N);
    for(uint i=0;i<phi_x.N;i++){
      if(            tt_x(i)==fTT                    ) coeff(i) += 1.;              // direct cost term
      if(            tt_x(i)==sumOfSqrTT             ) coeff(i) += 2.* phi_x(i);    // sumOfSqr terms
      if(muLB     && tt_x(i)==ineqTT                 ) coeff(i) -= (muLB/phi_x(i)); //log barrier, check feasibility
      if(mu       && tt_x(i)==ineqTT && I_lambda_x(i)) coeff(i) += 2.*mu*phi_x(i);  //g-penalty
      if(lambda.N && tt_x(i)==ineqTT && lambda(i)>0. ) coeff(i) += lambda(i);       //g-lagrange terms
      if(nu       && tt_x(i)==eqTT                   ) coeff(i) += 2.*nu*phi_x(i);  //h-penalty
      if(lambda.N && tt_x(i)==eqTT                   ) coeff(i) += lambda(i);       //h-lagrange terms
    }
    dL = comp_At_x(J_x, coeff);
    dL.reshape(x.N);

  }

  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values);

  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values){
    if(&HL){ //L hessian: Most terms are of the form   "J^T  diag(coeffs)  J"
      arr coeff=zeros(phi_x.N);
      int fterm=-1;
      for(uint i=0;i<phi_x.N;i++){
        if(            tt_x(i)==fTT){ if(fterm!=-1) HALT("There must only be 1 f-term (in the current implementation)");  fterm=i; }
        if(            tt_x(i)==sumOfSqrTT             ) coeff(i) += 2.;      // sumOfSqr terms
        if(muLB     && tt_x(i)==ineqTT                 ) coeff(i) += (muLB/mlr::sqr(phi_x(i)));  //log barrier, check feasibility
        if(mu       && tt_x(i)==ineqTT && I_lambda_x(i)) coeff(i) += 2.*mu;   //g-penalty
        if(nu       && tt_x(i)==eqTT                   ) coeff(i) += 2.*nu;   //h-penalty
      }
      arr tmp = J_x;
      for(uint i=0;i<phi_x.N;i++) tmp[i]() *= sqrt(coeff(i));
      HL = comp_At_A(tmp); //Gauss-Newton type!

      if(fterm!=-1){ //For f-terms, the Hessian must be given explicitly, and is not \propto J^T J
        HL += H_x;
      }

      if(!HL.special) HL.reshape(x.N,x.N);
    }

  }


  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
                                 const IpoptData* ip_data,
                                 IpoptCalculatedQuantities* ip_cq);
};
