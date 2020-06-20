#include <subtree_generators.h>

namespace mp
{
/// SUBTREES AFTER FIRST BRANCHING (QMDP)
SubTreesAfterFirstBranching::SubTreesAfterFirstBranching(const TreeBuilder& tree)
  : tree(tree)
{
  uint branching_node = 0;

  std::list<uint> queue;
  queue.push_back(0);

  while(!queue.empty())
  {
    auto p = queue.back();
    queue.pop_back();

    path_to_source.push_back(p);

    auto children = tree.get_children(p);

    if(children.size() > 1)
    {
      sources = children;
      break;
    }

    for(const auto& c: children)
    {
      queue.push_back(c);
    }
  }
}

bool SubTreesAfterFirstBranching::finished() const
{
  return (index == sources.size());
}

TreeBuilder SubTreesAfterFirstBranching::next()
{
  CHECK(!finished(), "finished generator");

  auto sub = tree.get_subtree_from(sources[index]);

  for(auto i = 1; i < path_to_source.size(); ++i)
  {
    sub.add_edge(path_to_source[i-1], path_to_source[i]);
  }

  sub.add_edge(path_to_source.back(), sources[index++]);

  return sub;
}

/// LINEAR SPLIT
LinearSplit::LinearSplit(const TreeBuilder& tree, uint n)
  : tree(tree)
  , n(n)
{
  std::list<uint> queue;
  queue.push_back(0);

  while(!queue.empty())
  {
    auto p = queue.back();
    queue.pop_back();

    for(const auto& q: tree.get_children(p))
    {
      splits.push_back(std::pair<uint, uint>(p, q));
      queue.push_back(q);
    }
  }
}

bool LinearSplit::finished() const
{
  return (index == splits.size());
}

TreeBuilder LinearSplit::next()
{
  auto pq = splits[index++];
  auto from = pq.first;
  auto to = pq.second;
  auto p = tree.p(0, to);

  TreeBuilder sub(p);
  sub.add_edge(from, to);
  return sub;
}

}
