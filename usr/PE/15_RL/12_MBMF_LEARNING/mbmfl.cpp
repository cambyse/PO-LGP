#include "mbmfl.h"

MBMFL::MBMFL(mlr::String name_):name(name_)
{
}

void MBMFL::saveMBMFL(){
  String n;
  n<< "data/"<<name<<"/";
  for (uint i=0;i<data_traj.d0;i++){
    write(LIST<arr>(data_traj(i)),STRING(n<<"data_traj"<<i));
  }
  write(LIST<arr>(data_param),STRING(n<<"data_param"));
  write(LIST<arr>(data_result),STRING(n<<"data_result"));
  write(LIST<arr>(ARR(data_traj.d0)),STRING(n<<"options"));
}

void MBMFL::loadMBMFL(){
  String n;
  n<< "data/"<<name<<"/";
  arr options = FILE(STRING(n<<"options"));
  data_traj.resize(options(0));
  data_param.resize(options(0));
  for (uint i=0;i<options(0);i++){
    data_traj(i) = FILE(STRING(n<<"data_traj"<<i));
  }
  data_param = FILE(STRING(n<<"data_param"));
  data_result = FILE(STRING(n<<"data_result"));
//  String n;
//  n<<"data/";
//  n<<String(name);
//  n <<"x";
//  x << FILE(n);
//  n <<"l";
//  lambda << FILE(n);
  //  lambda.flatten();
}

void MBMFL::addDatapoint(arr X,arr param, bool Y,double c)
{
  if (data_traj.N ==0){
    data_param.resize(1,param.d0);
    data_param[0] = param;
  }else{
    data_param.append(param);
  }

  data_traj.append(X);
  data_result.append(Y);
  data_cost.append(ARR(c));
//  arr A;
//  getAcc(A,X,1.);
//  data_cost.append(sumOfSqr(A));
}


