#pragma once

#include <deque>
#include <queue>
#include <unordered_set>
#include <memory>
#include <cstdlib>
#include <limits>
#include <list>

template <typename AstarNodeType>
struct HighestPriority
{
  bool operator()(const std::shared_ptr<AstarNodeType>& a, const std::shared_ptr<AstarNodeType>& b) const
  {
      return a->cost + a->to_go > b->cost + b->to_go;
  }
};

template <typename NodeType>
double heuristic(const std::shared_ptr<NodeType> & node, const std::list<std::shared_ptr<NodeType>>& final_nodes)
{
  auto d_min = std::numeric_limits<double>::infinity();

  for(const auto& n: final_nodes)
  {
      d_min = std::min(d_min, norm2(n->state, node->state));
  }

  return d_min;
};

template <typename GraphNodeType>
std::deque<typename GraphNodeType::State> astar(const std::shared_ptr<GraphNodeType>& root, const std::list<std::shared_ptr<GraphNodeType>>& final_nodes)
{
  using State = typename GraphNodeType::State;

  if(final_nodes.empty())
    return std::deque<State>();

  // prepare a star
  std::unordered_set<uint> final_ids;
  for(const auto& final: final_nodes)
  {
    final_ids.insert(final->id);
  }

  struct AstarNode
  {
    AstarNode(const std::shared_ptr<GraphNodeType> & node,
              double cost,
              double to_go,
              const std::shared_ptr<AstarNode> & parent)
      : node(node)
      , cost(cost)
      , to_go(to_go)
      , parent(parent)
    {

    }

    std::shared_ptr<GraphNodeType> node;
    double cost;
    double to_go;
    std::shared_ptr<AstarNode> parent;
  };



  // launch a-star
  std::priority_queue<
      std::shared_ptr<AstarNode>,
      std::vector<std::shared_ptr<AstarNode>>,
      HighestPriority<AstarNode>> q;
  std::unordered_set<uint> visited;

  std::shared_ptr<AstarNode> leaf;

  auto a_root = std::make_shared<AstarNode>(root, 0, heuristic(root, final_nodes), nullptr);
  q.push(a_root);
  while(!q.empty() && !leaf)
  {
    auto p = q.top();
    q.pop();

    // admissible heuristic
    assert(p->cost + p->to_go >= a_root->to_go);
    //

    for(const auto & c: p->node->children)
    {
      if(final_ids.find(c->id) != final_ids.end())
      {
        leaf = std::make_shared<AstarNode>(c, p->cost + norm2(p->node->state, c->state), heuristic(c, final_nodes), p);
        break;
      }
      else
      {
        if(visited.find(c->id) == visited.end())
        {
          visited.insert(c->id);
          q.push(std::make_shared<AstarNode>(c, p->cost + norm2(p->node->state, c->state), heuristic(c, final_nodes), p));
        }
      }
    }
  }

  // get path
  auto current = leaf;
  std::deque<State> path;
  path.push_back(current->node->state);
  while(current->parent)
  {
    path.push_front(current->parent->node->state);
    current = current->parent;
  }

  return path;
}
