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

MCTSPlanner::MCTSPlanner()
  : terminated_( false )
{

}

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
  { // no existing skeleton
    solveFirstTime();
  }
  else if( ! solved() )
  { // all existing skeletons are infeasible
    CHECK( ! (*solutions_.begin())->feasible(), "incoherent state of the solutions" );

    solveFirstTime();
  }
  else
  {
    generateAlternative();
  }

  // build policy
  PolicyBuilder builder( root_ );
  solutions_.push_back( builder.getPolicy() );
  solutions_.sort( policyCompare );

  // print
  PrintRewardsVisitor printer;
  root_->acceptVisitor( printer );
}

void MCTSPlanner::integrate( const Policy::ptr & policy )
{ 
  // keep the list sorted
  solutions_.sort( policyCompare );

  if( ! ( *solutions_.begin() )->feasible() )
  {
    // the policy in infeasible, the backtracking
    labelIfInfeasible( root_, policy->root() );
  }
  else
  {
    // is the policy the best?
    if( policy == ( *solutions_.begin() ) )
    {
      std::cout << "**********************" << std::endl;
      std::cout << "improved policy found!" << std::endl;
      std::cout << "**********************" << std::endl;

      // initialize fringes
      initFringes();
    }
    else
    {
      std::cout << "**********************" << std::endl;
      std::cout << "policy doesn't improve, switch back to backup!" << std::endl;
      std::cout << "**********************" << std::endl;

      switchBackToBackup();
    }
  }

  /*if( policy->feasible() )
  {

  else
  {
    // the policy in infeasible, the backtracking, generates the mutation!
    integrateFromNode( root_, policy->root() );

    policyMutated_ = true;
  }


  // if this is the best policy, noting to do
  /*solutions_.sort( policyCompare );

  if( policy->cost() == (*solutions_.begin())->cost() ) // the last policy is the best
  {
    std::cout << "improved policy found!" << std::endl;
  }
  // otherwise, switch back to the last best policy
  else
  {
    CHECK( alternativeStartNode_ !=  nullptr, "the backed up policy is invalid" );
    CHECK( nextFamilyBackup_.d0 > 0, "the backed up policy is invalid" );
    CHECK( nextFamilyBackup_( 0 )->parent() ==  alternativeStartNode_, "the backed up policy is invalid" );

    if( policy->cost() < std::numeric_limits< double >::infinity() )
    {
      CHECK( nextFamilyBackup_ != alternativeStartNode_->bestFamily(), "the tested action is the same than the back up one!" )

      currentPolicyFringe_ = currentPolicyFringeBackup_;
      currentPolicyFringeInitialized_ = true;
      alternativeStartNode_->setBestFamily( nextFamilyBackup_ );
    }
  }*/
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

void MCTSPlanner::labelIfInfeasible( const PONode::ptr & searchTreeNode, const PolicyNode::ptr &  policyNode )
{
  auto searchFamily = searchTreeNode->bestFamily();
  auto policyFamily = policyNode->children();

  CHECK( searchFamily.N == policyFamily.N, "error when reapplying the informed policy into the search tree" );

  for( auto i = 0; i < searchFamily.N; ++i )
  {
    auto searchNode = searchFamily( i );
    auto policyNode = policyFamily( i );

    CHECK( searchNode->id() == policyNode->id(), "error when reapplying the informed policy into the search tree, the node ids are invalid" )

    if( policyNode->g() == std::numeric_limits< double >::infinity() )
    {
      searchNode->labelInfeasible(); // changes the current policy via backtraking
    }

    labelIfInfeasible( searchNode, policyNode );
  }
}

void MCTSPlanner::solveFirstTime()
{
  std::cout << "MCTSPlanner::solveFirstTime.." << std::endl;

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
  std::cout << "MCTSPlanner::generateAlternative " << "policy fringe size:" << currentBestPolicyFringe.size() << std::endl;

  // get one node of the fringe, set it as better choice and solve from it
  if( currentBestPolicyFringe.size() > 0 )
  {
    //////here define a strategy to choose the action to change
    // for the moment just take the last one
    //auto alternativeFamily = *std::prev( currentBestPolicyFringe.end() );

    // take a random one
    auto j = ( currentBestPolicyFringe.size() * (double) rand() / (RAND_MAX));
    auto alternativeFamily = *std::next( currentBestPolicyFringe.begin(), j );

    auto branchingNode = alternativeFamily( 0 )->parent();

    CHECK( alternativeFamily != branchingNode->bestFamily(), "the tested family is he same than the back-up one!" );

    std::cout << "generating alternative starting from node:" << branchingNode->id() << std::endl;
    //////

    // solve alternative family
    uint s = 0;
    for( auto alternativeNode : alternativeFamily )
    {
      while( ! alternativeNode->isSolved() )
      {
        s++;

        auto nodes = getNodesToExpand( alternativeNode );

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

//          {
//            // save the current state of the search
//            std::stringstream namess;
//            namess << "exploration-alternative-" << alternativeNumber_ << "-" << s << ".gv";
//            printSearchTree( namess.str() );
//          }

          // backtrack result
          node->backTrackBestExpectedPolicy( alternativeNode );
        }
      }
    }

    // backup old policy
    currentBestPolicyFringe.erase( alternativeFamily );
    nextFamilyBackup_     = branchingNode->bestFamily();
    branchingNode->setBestFamily( alternativeFamily );
    //
    CHECK( nextFamilyBackup_ != branchingNode->bestFamily(), "the tested family is he same than the back-up one!" );
    CHECK( nextFamilyBackup_ != alternativeFamily, "the tested family is he same than the back-up one!" );
    //

    terminated_ = currentBestPolicyFringe.size() == 0;
  }
}

void MCTSPlanner::initFringes()
{
  utility::gatherPolicyFringe( root_, currentBestPolicyFringe );
}

void MCTSPlanner::switchBackToBackup()
{
  std::cout << "size of back up fringe:" << currentBestPolicyFringe.size() << std::endl;

  auto branchingNode = nextFamilyBackup_( 0 )->parent();
  branchingNode->setBestFamily( nextFamilyBackup_ );
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

void MCTSPlanner::checkIntegrity()
{
//  bool currentPolicyFringeInitialized_;
//  std::set< PONode::L > currentPolicyFringe_;
//  std::set< PONode::L > currentPolicyFringeBackup_;

//  PONode::ptr alternativeStartNode_;
//  PONode::L nextFamilyBackup_;

//  if(  )
}

}
