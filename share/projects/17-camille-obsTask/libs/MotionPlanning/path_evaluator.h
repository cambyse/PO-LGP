#pragma once

#include <Kin/kin.h>

namespace mp
{

std::pair< double, double > evaluate( mlr::Array< mlr::KinematicWorld > & kinFrames, double tau );

}
