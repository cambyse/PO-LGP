#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_djikstra
#include <boost/test/unit_test.hpp>

#include <graph_search.h>
#include <yens.h>
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
//  {
//    std::ofstream file;
//    file.open( skename );
//    policy->save( file );
//    file.close();
//  }
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

BOOST_AUTO_TEST_CASE( test_policy_clone )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-2w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  tp::Dijkstra dij( fols );

  auto pol_0 = dij.solve( graph, graph->root() );
  auto pol_00 = pol_0->clone();

  BOOST_CHECK( pol_0->N() == pol_00->N() );
  BOOST_CHECK( pol_0->leafs().size() == pol_00->leafs().size() );
  BOOST_CHECK( pol_0->cost() == pol_00->cost() );

  savePolicyToFile( pol_00 );
}

BOOST_AUTO_TEST_CASE( test_fuse_policies )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-2w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  tp::Dijkstra dij( fols );

  auto pol_0 = dij.solve( graph, graph->root() );
  savePolicyToFile( pol_0 );

  // remove 2->4, there is still a solution
  auto mask = std::make_shared< tp::GraphEdgeRewards >( graph );
  mask->removeEdge( 2, 4 );

  auto pol_1 = dij.solve( graph, graph->getNode( 2 ), mask );
  savePolicyToFile( pol_1 );

  auto pol_fused = fuse( pol_0, pol_1 );
  savePolicyToFile( pol_fused );
}

BOOST_AUTO_TEST_CASE( test_yens_0 )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-1w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  tp::Yens yens( fols );

  // solve with k = 0, or 1, we should get the first result of dijkstra
  auto policies = yens.solve( graph, 0 );

  BOOST_CHECK( policies.size() == 1 );

  policies = yens.solve( graph, 1 );

  BOOST_CHECK( policies.size() == 1 );

  // solve with k = 2, we should get two policies
  policies = yens.solve( graph, 3 );

  for( auto policy : policies )
  {
    savePolicyToFile( policy );
  }
}

BOOST_AUTO_TEST_CASE( test_yens_1 )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-2w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  tp::Yens yens( fols );

  // solve with k = 0, or 1, we should get the first result of dijkstra
  auto policies = yens.solve( graph, 0 );

  BOOST_CHECK( policies.size() == 1 );

  policies = yens.solve( graph, 1 );

  BOOST_CHECK( policies.size() == 1 );

  // solve with k = 2, we should get two policies
  policies = yens.solve( graph, 3 );

  for( auto policy : policies )
  {
    savePolicyToFile( policy );
  }
}

BOOST_AUTO_TEST_CASE( test_yens_2 )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-overtaking-3w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  tp::Yens yens( fols );

  // solve with k = 0, or 1, we should get the first result of dijkstra
  auto policies = yens.solve( graph, 0 );

  BOOST_CHECK( policies.size() == 1 );

  policies = yens.solve( graph, 1 );

  BOOST_CHECK( policies.size() == 1 );

  // solve with k = 2, we should get two policies
  policies = yens.solve( graph, 3 );

  for( auto policy : policies )
  {
    savePolicyToFile( policy );
  }
}
BOOST_AUTO_TEST_CASE( test_yens_3 )
{
  // tests that Dijkstra works well from root and from other nodes
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  tp->setFol( "data/LGP-blocks-fol-easy-1w.g" );
  tp->buildGraph();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto fols  = tp->getFolEngines();
  auto graph = tp->getGraph();

  tp::Yens yens( fols );

  // solve with k = 0, or 1, we should get the first result of dijkstra
  auto policies = yens.solve( graph, 0 );

  BOOST_CHECK( policies.size() == 1 );

  policies = yens.solve( graph, 1 );

  BOOST_CHECK( policies.size() == 1 );

  // solve with k = 2, we should get two policies
  policies = yens.solve( graph, 3 );

  for( auto policy : policies )
  {
    savePolicyToFile( policy );
  }
}
