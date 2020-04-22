#pragma once

#include <graph_planner.h>
#include <komo_factory.h>

void groundTreePickUp( double start, const mp::Vars& branch, double p, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreeUnStack( double start, const mp::Vars& branch, double p, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreePutDown( double start, const mp::Vars& branch, double p, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreeCheck( double start, const mp::Vars& branch, double p, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
void groundTreeStack( double start, const mp::Vars& branch, double p, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose );
