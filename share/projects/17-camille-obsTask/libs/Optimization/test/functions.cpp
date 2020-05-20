#include <Optim/newton.h>
#include <Optim/constrained.h>
#include <decentralized_aula.h>

struct Parabol : public ConstrainedProblem
{
  // min x^2
  // s.t. x < -0.5
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(2);
      J = arr(2, 1);
      ot = ObjectiveTypeA(2);

      ot(0) = OT_sos;
      ot(1) = OT_ineq;
    }

    phi(0) = x(0);
    J(0, 0) = 1.0;

    phi(1) = x(0) + 0.5;
    J(1, 0) = 1.0;
  }
};


struct Distance2D : public ConstrainedProblem
{
  // min dist to point (10, 2)
  // s.t. x = 0.0
  // s.t  y < 1
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(4);
      J = zeros(4, 2);
      ot = ObjectiveTypeA(4);

      ot(0) = OT_sos;
      ot(1) = OT_sos;
      ot(2) = OT_eq;
      ot(3) = OT_ineq;
    }

    phi(0) = x(0) - 10.0;
    J(0, 0) = 1.0;

    phi(1) = x(1) - 2.0;
    J(1, 1) = 1.0;

    phi(2) = x(0);
    J(2, 0) = 1.0;

    phi(3) = x(1) - 1.0;
    J(3, 1) = 1.0;
  }
};

struct ParabolWithFTerm : public ConstrainedProblem
{
  // min x^2 -x
  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(2);
      J = arr(2, 1);
      H = zeros(1, 1);
      ot = ObjectiveTypeA(2);

      ot(0) = OT_sos;
      ot(1) = OT_f;
    }

    phi(0) = x(0);
    J(0, 0) = 1.0;

    phi(1) = -x(0);
    J(1, 0) = -1.0;
  }
};

struct Distance3D : public ConstrainedProblem
{
  // min dist to center
  // s.t. x = 0.0
  // s.t  y < 1
  Distance3D(const arr& center, arr mask = ones(3))
    : center_(center)
    , mask_(mask)
  {
    CHECK_EQ(center.d0, 3, "wrong vector dimension")
    CHECK_EQ(mask.d0, 3, "wrong vector dimension")
  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(4);
      J = zeros(4, 3);
      ot = ObjectiveTypeA(4);

      ot(0) = OT_sos;
      ot(1) = OT_sos;
      ot(2) = OT_sos;
      ot(3) = OT_eq;
      //ot(4) = OT_ineq;
    }

    phi(0) = mask_(0) * (x(0) - center_(0));
    J(0, 0) = mask_(0);

    phi(1) = mask_(1) * (x(1) - center_(1));
    J(1, 1) = mask_(1);

    phi(2) = mask_(2) * (x(2) - center_(2));
    J(2, 2) = mask_(2);

    phi(3) = x(0);
    J(3, 0) = 1.0;

//    phi(4) = x(1) - 1.0;
//    J(4, 1) = 1.0;
  }

  arr center_;
  arr mask_;
};

struct Distance4D : public ConstrainedProblem
{
  // min dist to center
  // s.t. x = 0.0
  // s.t  y < 1
  Distance4D(const arr& center, arr mask = ones(4))
    : center_(center)
    , mask_(mask)
  {
    CHECK_EQ(center.d0, 4, "wrong vector dimension")
    CHECK_EQ(mask.d0, 4, "wrong vector dimension")
  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(5);
      J = zeros(5, 4);
      ot = ObjectiveTypeA(5);

      ot(0) = OT_sos;
      ot(1) = OT_sos;
      ot(2) = OT_sos;
      ot(3) = OT_sos;
      ot(4) = OT_eq;
      //ot(4) = OT_ineq;
    }

    phi(0) = mask_(0) * (x(0) - center_(0));
    J(0, 0) = mask_(0);

    phi(1) = mask_(1) * (x(1) - center_(1));
    J(1, 1) = mask_(1);

    phi(2) = mask_(2) * (x(2) - center_(2));
    J(2, 2) = mask_(2);

    phi(3) = mask_(3) * (x(3) - center_(3));
    J(3, 3) = mask_(3);

    phi(4) = x(0);
    J(4, 0) = 1.0;

//    phi(4) = x(1) - 1.0;
//    J(4, 1) = 1.0;
  }

  arr center_;
  arr mask_;
};
