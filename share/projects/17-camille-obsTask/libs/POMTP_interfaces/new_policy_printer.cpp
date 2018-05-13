#include <new_policy_printer.h>

#include <boost/algorithm/string/replace.hpp>

void NewPolicyPrinter::print( const NewPolicy & policy )
{
  if( ! policy.root() )
  {
    return;
  }

  ss_ << "digraph g{" << std::endl;
  ss_ << "bgcolor=\"transparent\"";
  ss_ << "{" << std::endl;
  ss_ << policy.root()->id() << " [style=filled, fillcolor=blue]" << std::endl;

//  for( auto weakN : policy.nodes() )
//  {
//    auto n = weakN.lock();

//    if( n )
//    {
//      if( n->data().agentId == 0 )
//      {
//        ss_ << n->id() << " [shape=square, style=filled, fillcolor=" << ( n->id() == 0 ? "blue" : "cyan" ) << "]" << std::endl;
//      }
//      else
//      {
//        ss_ << n->id() << " [shape=circle]" << std::endl;
//      }

//      if( n->data().nodeType == NodeData::NodeType::OBSERVATION )
//      {
//        ss_ << n->id() << " [shape=diamond]" << std::endl;
//      }
//    }
//  }

  for( auto weakN : policy.leafs() )
  {
    auto n = weakN.lock();

    if( n )
    {
      ss_ << n->id() << " [style=filled, fillcolor=green]" << std::endl;
    }
  }
  ss_ << "}" << std::endl;

  saveGraphFrom( policy.root() );

  ss_ << "}" << std::endl;
}

void NewPolicyPrinter::saveGraphFrom( const NewPolicy::GraphNodeType::ptr & node )
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
