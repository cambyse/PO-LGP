#include <graph_printer.h>

namespace tp {

void GraphPrinter::print( const POGraphNode::ptr & node, std::list< POGraphNode::ptr > terminals )
{
  if( ! node )
  {
    return;
  }

  ss_ << "digraph g{" << std::endl;
  ss_ << "bgcolor=\"transparent\"";
  ss_ << "{" << std::endl;
  ss_ << node->id() << " [style=filled, fillcolor=blue]" << std::endl;
  for( auto t : terminals )
  {
    ss_ << t->id() << " [style=filled, fillcolor=green]" << std::endl;
  }
  ss_ << "}" << std::endl;

  saveGraphFrom( node );

  ss_ << "}" << std::endl;
}

void GraphPrinter::saveGraphFrom( const POGraphNode::ptr & node )
{
  for( auto f : node->families() )
  {
    for( auto c : f )
    {
      std::pair< POGraphNode::ptr, POGraphNode::ptr > edge( node, c );

      if( std::find( savedEdges_.begin(), savedEdges_.end(), edge ) == savedEdges_.end() )
      {
        saveEdge( node, c );

        saveGraphFrom( c );
      }
    }
  }
}

void GraphPrinter::saveEdge( const POGraphNode::ptr & a, const POGraphNode::ptr & b )
{
  std::stringstream ss;

  ss << b->getLeadingActionFromStr( a );

  auto label = ss.str();

  ss_ << a->id() << "->" << b->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

  savedEdges_.push_back( std::pair< POGraphNode::ptr, POGraphNode::ptr >( a, b ) );
}

}
