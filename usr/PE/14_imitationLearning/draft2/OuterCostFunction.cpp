#include "OuterCostFunction.h"

SquaredDistanceOCF::SquaredDistanceOCF() {

}

double SquaredDistanceOCF::eval(const Demonstration &demA, const Demonstration &demB); {
return = sum((demA.qTraj - demB.qTraj)%(demA.qTraj - demB.qTraj));
}
