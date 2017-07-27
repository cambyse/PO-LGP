/*  ------------------------------------------------------------------
    Copyright 2016 Camille Phiquepal
    email: camille.phiquepal@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <mcts_planner.h>
#include <policy_builder.h>

namespace tp
{

void MCTSPlanner::setFol( const std::string & folDescription )
{
  const mlr::String notObservableTag = "NOT_OBSERVABLE";

  Graph KB;
  KB.read( FILE( folDescription.c_str() ) );

  // fully observable case
  if( KB[ beliefStateTag_ ] == nullptr )
  {
    // create dummy array
    folWorlds_ = mlr::Array< std::shared_ptr<FOL_World> > ( 1 );
    std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
    fol->init(FILE(folDescription.c_str()));
    folWorlds_( 0 ) = fol;
    fol->reset_state();
    // create dummy bs in observable case
    bs_ = arr( 1 );
    bs_( 0 ) = 1.0;
  }
  // partially observable case
  else
  {
    // get number of possible worlds
    auto bsGraph = &KB.get<Graph>( beliefStateTag_ );
    const uint nWorlds = bsGraph->d0;

    // generate all the possible fol
    folWorlds_ = mlr::Array< std::shared_ptr<FOL_World> > ( nWorlds );
    bs_ = arr( nWorlds );
    for( uint w = 0; w < nWorlds; w++ )
    {
      // retrieve the facts of the belief state
      std::shared_ptr<FOL_World> fol = std::make_shared<FOL_World>();
      fol->init(FILE(folDescription.c_str()));
      auto n = bsGraph->elem(w);

      StringA fact;
      // add fact
      for( auto s : n->parents ) fact.append( s->keys.last() );
      //fol->addFact(fact);

      // tag this fact as not observable
      StringA notObservableFact; notObservableFact.append( notObservableTag );
      for( auto s : fact ) notObservableFact.append( s );

      fol->addFact(notObservableFact);
      fol->reset_state();

      //std::cout << *fol << std::endl; // tmp
      folWorlds_(w) = fol;
      bs_(w) = n->get<double>();
    }

    // check that the belief state sums to 1
    double total = 0;
    for( auto p : bs_ ) total += p;

    CHECK( total == 1.00, "wrong belief state definition, the total of the probabilities doesn't sum to 1" );
  }

  root_ = std::make_shared< PONode >( folWorlds_, bs_ );
}

void MCTSPlanner::solve()
{
  std::cout << "MCTSPlanner::solveSymbolically" << std::endl;

  if( solutions_.empty() )
  {
    solveFirstTime();
  }
  else
  {
    generateAlternative();
  }

  // build policy
  PolicyBuilder builder( root_ );
  solutions_.insert( builder.getPolicy() );

  // print
  PrintRewardsVisitor printer;
  root_->acceptVisitor( printer );
}

void MCTSPlanner::integrate( const Policy::ptr & policy )
{

}

Policy::ptr MCTSPlanner::getPolicy() const
{
  Policy::ptr policy;

  if( solutions_.size() > 0 )
  {
    policy = *solutions_.begin();
  }

  return policy;
}

void MCTSPlanner::solveFirstTime()
{
  auto s = 0;
  while( ! solved() )
  {
    s++;
    auto nodes = getNodesToExpand();
    for( auto node : nodes )
    {
      // expand
      node->expand();

      // generate rollouts for each child
      for( auto f : node->families() )
      {
        for( auto c : f )
        {
          c->generateMCRollouts( 50, 10 );
        }
      }

      {
      // save the current state of the search
      //std::stringstream namess;
      //namess << "exploration-" << s << ".gv";
      //printSearchTree( namess.str() );
      }

      // backtrack result
      node->backTrackBestExpectedPolicy();
    }
  }
}

void MCTSPlanner::generateAlternative()
{

}

PONode::L MCTSPlanner::getNodesToExpand() const
{
  return getNodesToExpand( root_ );
}

PONode::L MCTSPlanner::getNodesToExpand( const PONode::ptr & node ) const
{
  PONode::L nodes;

  // starts from root
  if( ! node->isSolved() )
  {
    if( ! node->isExpanded() )
    {
      nodes.append( node );
    }
    else
    {
      for( auto c : node->bestFamily() )
      {
//        if( ! c->isExpanded() ) // this part of code can be safely commented because it does n'tbring anything ( the recursiondoes it anyway )
//        {
//          nodes.append( c );
//        }
//        else
//        {
          nodes.append( getNodesToExpand( c ) );
//        }
      }
    }
  }

  return nodes;
}

}
