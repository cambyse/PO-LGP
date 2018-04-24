#include <decision_graph_printer.h>

namespace matp
{

void GraphPrinter::print( const DecisionGraph & graph )
{
  ss_ << "digraph g{" << std::endl;
  ss_ << "bgcolor=\"transparent\"";
  ss_ << "{" << std::endl;
  ss_ << graph.root()->id() << " [style=filled, fillcolor=blue]" << std::endl;
  for( auto n : graph.nodes() )
  {
    if( n->data().agentId == 0 )
    {
      ss_ << n->id() << " [shape=square, style=filled, fillcolor=" << ( n->id() == 0 ? "blue" : "cyan" ) << "]" << std::endl;
    }
    else
    {
      ss_ << n->id() << " [shape=circle]" << std::endl;
    }

    if( n->data().nodeType == NodeData::NodeType::OBSERVATION )
    {
      ss_ << n->id() << " [shape=diamond]" << std::endl;
    }
  }
  for( auto n : graph.terminalNodes() )
  {
    ss_ << n->id() << " [style=filled, fillcolor=green]" << std::endl;
  }
  ss_ << "}" << std::endl;

  saveGraphFrom( graph.root() );

  ss_ << "}" << std::endl;
}

void GraphPrinter::saveGraphFrom( const DecisionGraph::GraphNodeType::ptr & node )
{
  for(  auto c : node->children() )
  {
    std::stringstream ss;

    auto label = c->data().leadingArtifact;

    ss << c->data().leadingArtifact << std::endl;;
    auto label = ss.str();

    ss_ << node->id() << "->" << c->id() << " [ label=\"" << label << "\" ]" << ";" << std::endl;

    saveGraphFrom( c );
  }
}

}
