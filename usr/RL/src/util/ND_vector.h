#ifndef ND_VECTOR_H_
#define ND_VECTOR_H_

namespace ND_vector {

    // aliases

    template <class T>
        using vec_1D = std::vector<T>;

    template <class T>
        using vec_2D = std::vector<vec_1D<T>>;

    template <class T>
        using vec_3D = std::vector<vec_2D<T>>;

    // template maker function
    template <class T>
        vec_1D<T> make_vec_1D(int d1, T val = T()) {
        return vec_1D<T>(d1, val);
    }

    template <class T>
        vec_2D<T> make_vec_2D(int d1, int d2, T val = T()) {
        return vec_2D<T>(d1, make_vec_1D(d2, val));
    }

    template <class T>
        vec_3D<T> make_vec_3D(int d1, int d2, int d3, T val = T()) {
        return vec_3D<T>(d1, make_vec_2D(d2, d3, val));
    }

    // typedefs for specific types
    typedef vec_1D<int> vec_int_1D;
    typedef vec_2D<int> vec_int_2D;
    typedef vec_3D<int> vec_int_3D;

    typedef vec_1D<double> vec_double_1D;
    typedef vec_2D<double> vec_double_2D;
    typedef vec_3D<double> vec_double_3D;

    // maker functions for specific types
    inline vec_int_1D make_vec_int_1D(int d1, int val = 0) { return make_vec_1D<int>(d1, val); }
    inline vec_int_2D make_vec_int_2D(int d1, int d2, int val = 0) { return make_vec_2D<int>(d1, d2, val); }
    inline vec_int_3D make_vec_int_3D(int d1, int d2, int d3, int val = 0) { return make_vec_3D<int>(d1, d2, d3, val); }

    inline vec_double_1D make_vec_double_1D(double d1, double val = 0) { return make_vec_1D<double>(d1, val); }
    inline vec_double_2D make_vec_double_2D(double d1, double d2, double val = 0) { return make_vec_2D<double>(d1, d2, val); }
    inline vec_double_3D make_vec_double_3D(double d1, double d2, double d3, double val = 0) { return make_vec_3D<double>(d1, d2, d3, val); }

} // end namespace ND_vector

#endif /* ND_VECTOR_H_ */
