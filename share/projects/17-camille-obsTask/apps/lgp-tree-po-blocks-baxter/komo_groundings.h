#pragma one

#include <graph_planner.h>

//==========Application specific grounders===================================

void groundInit( KOMO_ext * komo, int verbose );
void groundPickUp( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose );
void groundUnStack( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose );
void groundPutDown( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose );
void groundCheck( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose );
void groundStack( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose );
