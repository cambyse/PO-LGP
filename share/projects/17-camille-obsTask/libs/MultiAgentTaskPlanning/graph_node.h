#pragma once

#include <memory>
#include <list>

namespace matp
{

template < typename T >
class GraphNode : public std::enable_shared_from_this< GraphNode< T > >
{
public:
  using ptr = std::shared_ptr< GraphNode< T > >;
  using weak_ptr = std::weak_ptr< GraphNode< T > >;

private:
  GraphNode( const T & data )
    : id_( 0 )
    , depth_( 0 )
    , data_( data )
  {

  }

  GraphNode( const weak_ptr & parent, uint id, const T & data )
    : parent_( parent )
    , id_( id )
    , depth_( parent.lock()->depth_ + 1 )
    , data_( data )
  {

  }

public:
  static ptr root( const T & data ) { counter_ = 1; return ptr( new GraphNode< T >( data ) ); }

  bool isRoot() const { return parent_.lock() == nullptr; }
  std::list< ptr > children() const { return children_; }
  std::list< ptr > siblings() const
  {
    auto parent = parent_.lock();
    std::list< ptr > siblings;

    for( auto s : parent->children() )
    {
      if( s != this->shared_from_this() )
      {
        siblings.push_back( s );
      }
    }

    return siblings;
  }
  uint id() const { return id_; }
  uint depth() const { return depth_; }
  T data() const { return data_; }

  ptr makeChild( const T & data )
  {
    auto child = ptr( new GraphNode< T >( this->shared_from_this(), counter_, data ) );
    children_.push_back( child );

    counter_++;

    return child;
  }

  void addExistingChild( const ptr & child )
  {
    children_.push_back( child );
  }

  void clearChildren()
  {
    children_.clear();
  }
public:
  static uint counter_;
private:
  uint id_;
  uint depth_;
  weak_ptr parent_;
  T data_;
  std::list< ptr > children_;
};

template<typename T>
uint GraphNode<T>::counter_=0;

} // namespace matp
