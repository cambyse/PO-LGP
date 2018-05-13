#pragma once

#include <fstream>

#include <new_policy.h>

class NewPolicyPrinter
{
public:
  NewPolicyPrinter( std::ostream & ss )
    : ss_( ss )
  {

  }

  void print( const NewPolicy & policy );

private:
  void saveGraphFrom( const NewPolicy::GraphNodeType::ptr & node );

private:
  std::ostream & ss_;
};
