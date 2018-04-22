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
    , data_( data )
  {

  }

  GraphNode( const weak_ptr & parent, uint id, const T & data )
    : parent_( parent )
    , id_( id )
    , data_( data )
  {

  }

public:
  static ptr root( const T & data ) { return ptr( new GraphNode< T >( data ) ); }

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
  T data() const { return data_; }

  ptr makeChild( const T & data )
  {
    this->shared_from_this();
    auto child = ptr( new GraphNode< T >( this->shared_from_this(), id_+ 1, data ) );

    children_.push_back( child );

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

private:
  weak_ptr parent_;
  uint id_;
  T data_;
  std::list< ptr > children_;
};

} // namespace matp
