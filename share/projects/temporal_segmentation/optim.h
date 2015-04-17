#pragma once

struct ObjectiveFunction {
  uint arity;
  std::function<double(arr)> f;
  std::function<arr(arr)> jacobian, hessian;

  ObjectiveFunction() {};
  ObjectiveFunction(uint _arity): arity(_arity) {};
  ~ObjectiveFunction() {};

  void test(const arr &x) {
    if(f)
      cout << "f: " << f(x) << endl;
    if(jacobian)
      cout << "jacobian: " << jacobian(x) << endl;
    if(hessian)
      cout << "hessian: " << hessian(x) << endl;
  }
};

struct LineSearch {
  ObjectiveFunction objf;
  double rho_alpha_minus, rho_ls;

  LineSearch(): rho_alpha_minus(.5), rho_ls(.15) {}
  ~LineSearch() {}

  double search(const arr &x, const arr &d);
};

struct BFGS {
  ObjectiveFunction objf;
  arr invH;

  void loopUntil(arr &x, double end_condition);
};

struct Newton {
  ObjectiveFunction objf;

  void loopUntil(arr &x, double end_condition);
};

