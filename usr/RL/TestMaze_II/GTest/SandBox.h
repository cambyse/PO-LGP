#ifndef ENUMERATE_H_
#define ENUMERATE_H_

#include <utility>

template<typename T,
    template <typename, typename = std::allocator<T>> class Container>
class Enumerate {
    //----typedefs/classes----//
public:

    class Iterator {
        //----typedefs/classes----//
        //          NONE          //
        //----members----//
    private:
        typename Container<T>::iterator iter;
        int counter;

        //----methods----//
    public:
        Iterator(typename Container<T>::iterator i, int c = 0): iter(i), counter(c) {}
        virtual ~Iterator() = default;
        std::pair<int, T> operator*() {
            return std::make_pair(counter, *iter);
        }
        Iterator operator++() {
            ++counter;
            return Iterator(++iter, counter);
        }
        bool operator!=(const Iterator& it) {
            return this->iter!=it.iter;
        }
    };

    //----members----//
private:
    Container<T> & container;

    //----methods----//
public:
    Enumerate(Container<T> & c): container(c){}
    virtual ~Enumerate() = default;
    Iterator begin() {
        return Iterator(container.begin());
    }
    Iterator end() {
        return Iterator(container.end());
    }
};

template<typename T,
    template <typename, typename = std::allocator<T>> class Container>
    Enumerate<T, Container> enumerate(Container<T> c) {
    return Enumerate<T, Container>(c);
}

#endif /* ENUMERATE_H_ */
