#pragma once

#include <array>
#include <cstdlib>
#include <list>
#include <deque>
#include <queue>
#include <unordered_set>
#include <memory>
#include <functional>
#include <cassert>
#include <nearest_neighbor.h>
#include <sample_space.h>
#include <common.h>
#include <astar.h>

template <uint N, uint W>
struct PORRTStarNode
{
  std::weak_ptr<PORRTStarNode<N, W>> parent;
  std::array<double, N> state; // continuous state
  std::array<double, W> validities; // discrete state mask
  std::vector<std::shared_ptr<PORRTStarNode<N, W>>> children;
  uint id{0};

  static constexpr uint dim{N};
  typedef std::array<double, N> State;
};

template<uint N, uint W>
class PORRTStarTree
{
public:
  PORRTStarTree(const std::array<double, N> & root_state)
  {
    root_ = std::make_shared<PORRTStarNode<N, W>>();
    root_->state = root_state;
    nodes_.push_back(root_);
  }

  std::shared_ptr<PORRTStarNode<N, W>> get_node(uint id) const
  {
    return nodes_[id];
  }

  std::shared_ptr<PORRTStarNode<N, W>> add_node(const std::array<double, N> & state)
  {
    auto node = std::make_shared<PORRTStarNode<N, W>>();
    node->state = state;
    node->id = nodes_.size();
    nodes_.push_back(node);

    return node;
  }

  void add_edge(const std::shared_ptr<PORRTStarNode<N, W>> & from, const std::shared_ptr<PORRTStarNode<N, W>> & to) const
  {
    from->children.push_back(to);
    to->parent = from;
    //to->cost = from->cost + norm2(from->state, to->state);
  }

  const std::vector<std::shared_ptr<PORRTStarNode<N, W>>> & nodes() const
  {
    return nodes_;
  }

private:
  std::vector<std::shared_ptr<PORRTStarNode<N, W>>> nodes_;
  std::shared_ptr<PORRTStarNode<N, W>> root_;
};

template<typename S, uint W> // sample space
class PORRTStar
{
public:
  PORRTStar(const S & space, double max_step=1.0, double radius=1.0)
    : space_(space)
    , max_step_(max_step) // max tree expansion
    , radius_(radius)
  {

  }

  void set_state_checker(const std::function<std::array<bool, W>(const std::array<double, S::dim> &)> & state_checker)
  {
    state_checker_ = state_checker;
  }

  void set_transition_checker(const std::function<bool(const std::array<double, S::dim> &, const std::array<double, S::dim> &)> & transition_checker)
  {
    transition_checker_ = transition_checker;
  }

  std::deque<std::array<double, S::dim>> plan(const std::array<double, S::dim> & start,
                                              const std::function<bool(const std::array<double, S::dim> &)> & goal_cnd,
                                              uint n_iter_max)
  {
    grow_tree(start, goal_cnd, n_iter_max);

    return plan_path();
  }

  std::shared_ptr<PORRTStarTree<S::dim, W>> rrt_tree() const
  {
    return rrttree_;
  }

private:
  void grow_tree(const std::array<double, S::dim> & start,
                 const std::function<bool(const std::array<double, S::dim> &)> & goal_cnd,
                 uint n_iter_max)
  {
    assert(state_checker_);
    assert(transition_checker_);

    // grow tree
    rrttree_ = std::make_shared<PORRTStarTree<S::dim, W>>(start);
    kdtree_ = std::make_unique<KDTree<S::dim>>(start);

    for(uint i = 0; i < n_iter_max; ++i)
    {
      auto s = space_.sample();
      const auto& node = kdtree_->nearest_neighbor(s);

      backtrack(node->state, s, max_step_);

      auto validities = state_checker_(s);
      if(validities[0])
      {
        auto neighbors = kdtree_->radius_neighbors(s, radius_);

        std::shared_ptr<PORRTStarNode<S::dim, W>> new_rrt_node;

        for(const auto& kd_neighbor: neighbors)
        {
          if(transition_checker_(kd_neighbor->state, s))
          {
            const auto& neighbor = rrttree_->get_node(kd_neighbor->id);

            if(!new_rrt_node)
            {
              new_rrt_node = rrttree_->add_node(s);
              kdtree_->add_node(new_rrt_node->state, new_rrt_node->id);
            }

            rrttree_->add_edge(neighbor, new_rrt_node);
            //rrttree_->add_edge(new_rrt_node, neighbor);
          }
        }

        if(goal_cnd(new_rrt_node->state))
        {
          // found solution
          //if(final_nodes_.empty())
          final_nodes_.push_back(new_rrt_node);// break;
        }
      }
    }
  }

  std::deque<std::array<double, S::dim>> plan_path() const
  { 
    return astar(rrttree_->nodes()[0], final_nodes_);
  }

private:
  const S & space_;
  const double max_step_;
  const double radius_;
  std::shared_ptr<PORRTStarTree<S::dim, W>> rrttree_;
  std::unique_ptr<KDTree<S::dim>> kdtree_;
  std::function<std::array<bool, W>(const std::array<double, S::dim> &)> state_checker_; // indicates in which discrete state the sample is valid
  std::function<bool(const std::array<double, S::dim> &, const std::array<double, S::dim> &)> transition_checker_;
  std::list<std::shared_ptr<PORRTStarNode<S::dim, W>>> final_nodes_;
};
