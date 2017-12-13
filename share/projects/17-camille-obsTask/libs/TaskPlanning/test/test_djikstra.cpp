#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_djikstra
#include <boost/test/unit_test.hpp>

#include <graph_search.h>
#include <po_djikstra.h>
#include <policy_printer.h>

static void generatePngImage( const std::string & name )
{
  std::string nameCopy( name );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << name;

  system( ss.str().c_str() );
}

static void savePolicyToFile( const Policy::ptr & policy )
{
  if( ! policy )
    return;

  std::stringstream namess, skenamess;
  namess << "policy-" << policy->id() << ".gv";
  skenamess << "policy-" << policy->id() << ".ske";
  auto skename = skenamess.str();
  auto name = namess.str();

  // save full policy
  {
    std::ofstream file;
    file.open( skename );
    policy->save( file );
    file.close();
  }
  // generate nice graph
  {
    std::ofstream file;
    file.open( name );
    PolicyPrinter printer( file );
    printer.print( policy );
    file.close();

    generatePngImage( name );
  }
}

BOOST_AUTO_TEST_CASE( test_djikstra_0 )
{
  Graph KB1;
  KB1.read( FILE( "data/LGP-overtaking-2w.g" ) );

  Graph KB2;
  KB2.read( FILE( "data/LGP-overtaking-2w.g" ) );

  BOOST_CHECK( KB1 == KB2 );
}

BOOST_AUTO_TEST_CASE( test_djikstra_1 )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-2w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  // solve from root
  {
  tp::Dijkstra dij( fols );
  auto pol = dij.solve( graph, graph->root() );

  savePolicyToFile( pol );
  }
  // solve from 1
  {
  tp::Dijkstra dij( fols );
  auto pol = dij.solve( graph, graph->getNode( 1 ) );

  savePolicyToFile( pol );
  }
  // solve from 2
  {
  tp::Dijkstra dij( fols );
  auto pol = dij.solve( graph, graph->getNode( 2 ) );

  savePolicyToFile( pol );
  }
  // solve from 3
  {
  tp::Dijkstra dij( fols ); // no policy already terminal!
  auto pol = dij.solve( graph, graph->getNode( 3 ) );

  savePolicyToFile( pol );
  }
  // solve from 5
  {
  tp::Dijkstra dij( fols );
  auto pol = dij.solve( graph, graph->getNode( 5 ) );

  savePolicyToFile( pol );
  }
}

BOOST_AUTO_TEST_CASE( test_graph_build )
{
  // tests that Dijkstra works well from root and from other nodes when edges are removed
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-2w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph-2.gv" );
  generatePngImage( "graph-2.gv" );
}

BOOST_AUTO_TEST_CASE( test_djikstra_remove_edges )
{
  // tests that Dijkstra works well from root and from other nodes when edges are removed
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-2w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  // solve from root
  {
  // remove 2->4, there is still a solution
  auto mask = std::make_shared< tp::GraphEdgeRewards >( graph );
  mask->removeEdge( 2, 4 );

  tp::Dijkstra dij( fols );
  auto pol = dij.solve( graph, graph->root(), mask );

  savePolicyToFile( pol );
  }

  {
  // remove 0->1, no solution from root
  auto mask = std::make_shared< tp::GraphEdgeRewards >( graph );
  mask->removeEdge( 0, 1 );

  tp::Dijkstra dij( fols );
  auto pol = dij.solve( graph, graph->root(), mask );

  savePolicyToFile( pol );

  // but still a solution from 2!
  pol = dij.solve( graph, graph->getNode( 2 ), mask );

  savePolicyToFile( pol );

  }
}

BOOST_AUTO_TEST_CASE( test_djikstra_remove_node )
{
  // tests that Dijkstra works well from root and from other nodes when edges are removed
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-2w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  // solve from root
  {
  // remove 2->4, there is still a solution
  auto mask = std::make_shared< tp::GraphEdgeRewards >( graph );
  mask->removeNode( 4 );

  tp::Dijkstra dij( fols );
  auto pol = dij.solve( graph, graph->root(), mask );

  savePolicyToFile( pol );
  }
}
