#include <subtree_generators.h>
#include <unordered_set>

namespace mp
{

std::vector<Edge> interactingEdges(const TreeBuilder& tree, const TreeBuilder& subtree)
{
  std::vector<Edge> edges;

  auto subleaves = subtree.get_leaves();
  std::vector<uint> leaves;

  for(auto n: subleaves)
  {
    auto extended = tree.get_leaves_from(n);
    leaves.insert(leaves.begin(), extended.begin(), extended.end());
  }

  std::list<uint> lifo;
  std::unordered_set<uint> visited;
  for(const auto& l: leaves)
  {
    lifo.push_back(l);
  }

  while(!lifo.empty())
  {
    auto q = lifo.back();
    lifo.pop_back();

    if(visited.find(q) != visited.end())
      continue;

    auto ps = tree.get_parents(q);

    for(const auto p: ps)
    {
      visited.insert(q);
      lifo.push_back(p);

      edges.push_back(Edge({p, q}));
    }
  }

  std::reverse(edges.begin(), edges.end());

  return edges;
}

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

  CHECK(sources.size() > 0, "No first branching!, check that the policy is not linear!");
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
