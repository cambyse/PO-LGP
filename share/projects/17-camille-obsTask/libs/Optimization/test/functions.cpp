#include <Optim/newton.h>
#include <Optim/constrained.h>

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
  Distance3D(const arr& center)
    : center_(center)
  {
    CHECK_EQ(center.d0, 3, "wrong vector dimension")
  }

  void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x, arr& lambda)
  {
    if(!phi.p)
    {
      phi = arr(5);
      J = zeros(5, 3);
      ot = ObjectiveTypeA(5);

      ot(0) = OT_sos;
      ot(1) = OT_sos;
      ot(2) = OT_sos;
      ot(3) = OT_eq;
      ot(4) = OT_ineq;
    }

    phi(0) = x(0) - center_(0);
    J(0, 0) = 1.0;

    phi(1) = x(1) - center_(1);
    J(1, 1) = 1.0;

    phi(2) = x(2) - center_(2);
    J(2, 2) = 1.0;

    phi(3) = x(0);
    J(3, 0) = 1.0;

    phi(4) = x(1) - 1.0;
    J(4, 1) = 1.0;
  }

  arr center_;
};

struct Distance3DDecompXY : public ConstrainedProblem
{
  // version of Distance3D decomposed (doesn't consider z axis)
  // s.t. x = 0.0
  // s.t  y < 1
  Distance3DDecompXY(const arr& center)
    : center_(center)
    , scaleX_(sqrt(0.5))
  {
    CHECK_EQ(center.d0, 3, "wrong vector dimension")
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
      ot(2) = OT_eq;
      ot(3) = OT_ineq;
    }

    phi(0) = scaleX_ * (x(0) - center_(0));
    J(0, 0) = scaleX_ * 1.0;

    phi(1) = x(1) - center_(1);
    J(1, 1) = 1.0;

    phi(2) = x(0);
    J(2, 0) = 1.0;

    phi(3) = x(1) - 1.0;
    J(3, 1) = 1.0;
  }

  arr center_;
  double scaleX_;
};

struct Distance3DDecompXZ : public ConstrainedProblem
{
  // version of Distance3D decomposed (doesn't consider y axis)
  // s.t. x = 0.0
  // s.t  y < 1
  Distance3DDecompXZ(const arr& center)
    : center_(center)
    , scaleX_(sqrt(0.5))
  {
    CHECK_EQ(center.d0, 3, "wrong vector dimension")
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
      ot(2) = OT_eq;
      ot(3) = OT_ineq;
    }

    phi(0) = scaleX_ * (x(0) - center_(0));
    J(0, 0) = scaleX_ * 1.0;

    phi(1) = x(2) - center_(2);
    J(1, 2) = 1.0;

    phi(2) = x(0);
    J(2, 0) = 1.0;

    phi(3) = x(1) - 1.0;
    J(3, 1) = 1.0;
  }

  arr center_;
  double scaleX_;
};

