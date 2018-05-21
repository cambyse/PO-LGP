#pragma once

#include <fstream>

#include <skeleton.h>

class SkeletonPrinter
{
public:
  SkeletonPrinter( std::ostream & ss )
    : ss_( ss )
  {

  }

  void print( const Skeleton & policy );

private:
  void saveGraphFrom( const Skeleton::GraphNodeType::ptr & node );

private:
  std::ostream & ss_;
};
