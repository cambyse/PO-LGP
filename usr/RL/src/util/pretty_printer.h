#ifndef PRETTY_PRINTER_H_
#define PRETTY_PRINTER_H_

namespace util {

    namespace pretty_printer{
        template<std::size_t...> struct seq{};

        template<std::size_t N, std::size_t... Is>
            struct gen_seq : gen_seq<N-1, N-1, Is...>{};

        template<std::size_t... Is>
            struct gen_seq<0, Is...> : seq<Is...>{};

        template<class Ch, class Tr, class Tuple, std::size_t... Is>
            void print_tuple(std::basic_ostream<Ch,Tr>& os, Tuple const& t, seq<Is...>){
            using swallow = int[];
            (void)swallow{0, (void(os << (Is == 0? "" : ", ") << std::get<Is>(t)), 0)...};
        }
    }

    /** Convert a iterable container to a std::string. */
    template <class C>
        std::string container_to_str(const C & container,
                                     const char* separator = " ",
                                     const char* start = "",
                                     const char* end = "") {
        std::stringstream ss;
        bool first = true;
        ss << start;
        for(auto elem : container) {
            if(!first) {
                ss << separator;
            } else {
                first = false;
            }
            ss << elem;
        }
        ss << end;
        return ss.str();
    }
}

namespace std {
    /** Defines ostream operator for iterable containers. The ValueType template
     * parameter disambiguates (via SFNIAE) overloads. */
    template<class Ch, class Tr, class C, class ValueType = typename C::value_type>
        auto operator<<(std::basic_ostream<Ch, Tr>& os, const C & container) -> std::basic_ostream<Ch, Tr> & {
        os << util::container_to_str(container);
        return os;
    }

    /** Defines ostream operator for tuples. */
    template<class Ch, class Tr, class... Args>
        auto operator<<(std::basic_ostream<Ch, Tr>& os, std::tuple<Args...> const& t) -> std::basic_ostream<Ch, Tr>& {
        os << "(";
        util::pretty_printer::print_tuple(os, t, util::pretty_printer::gen_seq<sizeof...(Args)>());
        return os << ")";
    }
}

#endif /* PRETTY_PRINTER_H_ */
