#include <skeleton_printer.h>

#include <boost/algorithm/string/replace.hpp>

void SkeletonPrinter::print( const Skeleton & skeleton )
{
  if( ! skeleton.root() )
  {
    return;
  }

  ss_ << "digraph g{" << std::endl;
  ss_ << "bgcolor=\"transparent\"";
  ss_ << "{" << std::endl;
  ss_ << skeleton.root()->id() << " [style=filled, fillcolor=blue]" << std::endl;

  for( auto weakN : skeleton.leafs() )
  {
    auto n = weakN.lock();

    if( n )
    {
      ss_ << n->id() << " [style=filled, fillcolor=green]" << std::endl;
    }
  }
  ss_ << "}" << std::endl;

  saveGraphFrom( skeleton.root() );

  ss_ << "}" << std::endl;
}

void SkeletonPrinter::saveGraphFrom( const Skeleton::GraphNodeType::ptr & node )
{
  for( auto c : node->children() )
  {
    std::stringstream ss;
    std::string label;

    uint argIndex = 0;
    for( auto arg : c->data().leadingKomoArgs )
    {
      ss << arg;

      if( argIndex == 0 )
      {
        ss << "(";
      }
      else if( argIndex < c->data().leadingKomoArgs.size() - 1 )
      {
        ss << ",";
      }

      argIndex++;
    }

    ss << ")";

    ss << std::endl;

    //
    ss << "r=" << c->data().markovianReturn << std::endl;

    label = ss.str();
    boost::replace_all(label, "{", "");

    ss_ << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    saveGraphFrom( c );
  }
}