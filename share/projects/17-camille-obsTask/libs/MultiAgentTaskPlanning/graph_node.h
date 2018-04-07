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
    : data_( data )
  {

  }

  GraphNode( const weak_ptr & parent, const T & data )
    : parent_( parent )
    , data_( data )
  {

  }

public:
  static ptr root( const T & data ) { return ptr( new GraphNode< T >( data ) ); }

  bool isRoot() const { return parent_.lock() == nullptr; }
  std::list< ptr > children() const { return children_; }

  ptr makeChild( const T & data )
  {
    this->shared_from_this();
    auto child = ptr( new GraphNode< T >( this->shared_from_this(), data ) );

    children_.push_back( child );

    return child;
  }

private:
  weak_ptr parent_;
  T data_;
  std::list< ptr > children_;
};

} // namespace matp
