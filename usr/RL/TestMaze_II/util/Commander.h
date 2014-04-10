#ifndef COMMANDER_H_
#define COMMANDER_H_

#include "function_signature.h"

#include <QString>
#include "QtUtil.h"

#include <algorithm> // std::reverse
#include <vector>
#include <tuple>
#include <typeinfo>
#include <typeindex>
#include <functional> // std::function
#include <memory> // std::shared_ptr
#include <unordered_map>

#include "debug.h"

namespace Commander {

    /** \brief Represents a list of command aliases. */
    class CommandAliasList {
        //----members----//
    private:
        std::vector<QString> commands;
        //----methods----//
    public:
        CommandAliasList(QString);
        CommandAliasList(const char*);
        CommandAliasList(std::initializer_list<QString>);
        virtual ~CommandAliasList() = default;
        bool contains(const QString& other) const;
        bool has_common_element(const CommandAliasList& other) const;
        bool operator<(const CommandAliasList& other) const { return commands<other.commands; }
        int length() const { return get_string().length(); }
        operator QString() const { return get_string(); }
    private:
        QString get_string() const;
    };

    /** \brief Mapper for types. */
    template<class From, class To> class Mapper {
    public:
        static const bool can_do = false;
        static To map(From f) { return To(); }
    };

    /** \brief Specialization for QString-->int. */
    template<> class Mapper<QString,std::tuple<bool,int>> {
    public:
        static const bool can_do = true;
        static std::tuple<bool,int> map(QString from) {
            bool ok;
            int i = from.toInt(&ok);
            return std::make_tuple(ok,i);
        }
    };

    /** \brief Specialization for QString-->double. */
    template<> class Mapper<QString,std::tuple<bool,double>> {
    public:
        static const bool can_do = true;
        static std::tuple<bool,double> map(QString from) {
            bool ok;
            double d = from.toDouble(&ok);
            return std::make_tuple(ok,d);

        }
    };

    /** \brief Specialization for QString-->QString. */
    template<> class Mapper<QString,std::tuple<bool,QString>> {
    public:
        static const bool can_do = true;
        static std::tuple<bool,QString> map(QString from) {
            return std::make_tuple(true,from);
        }
    };


    /** \brief Maps to a type description. */
    template<class From, class To> class TypeDescriptor {
    public:
        static const bool can_do = false;
        static To map(From f) { return To(); }
    };

    /** \brief Specialization for bool-->QString. */
    template<> class TypeDescriptor<bool,QString> {
    public:
        static const bool can_do = true;
        static QString map(bool) {
            return "<bool>";
        }
    };

    /** \brief Specialization for int-->QString. */
    template<> class TypeDescriptor<int,QString> {
    public:
        static const bool can_do = true;
        static QString map(int) {
            return "<int>";
        }
    };

    /** \brief Specialization for double-->QString. */
    template<> class TypeDescriptor<double,QString> {
    public:
        static const bool can_do = true;
        static QString map(double) {
            return "<double>";
        }
    };

    /** \brief Specialization for QString-->QString. */
    template<> class TypeDescriptor<QString,QString> {
    public:
        static const bool can_do = true;
        static QString map(QString) {
            return "<QString>";
        }
    };


    /** \brief Return type of command-functions.
     *
     * First element signals whether the received parameters were ok, second
     * element is an (error) message. */
    typedef std::pair<bool,QString> ReturnType;

    /** \brief Base type for command-functions. */
    class AbstractCommandFunction {
    public:
        QString arg_description;
        ReturnType wrong_number_of_parameters(int,int) const;
        virtual ReturnType execute(const QStringList&) const = 0;
        virtual int get_n_args() const = 0;
    };

    /** \brief Templatized version to store specific command-functions. */
    template<class Func>
        class CommandFunction : public AbstractCommandFunction {
    public:
        typedef function_signature::make_function_type<Func> func_t;
        func_t func;
        constexpr static int n_args = function_signature::get_n_args<Func>();
        CommandFunction(Func f);
        virtual ReturnType execute(const QStringList& args) const override;
        virtual int get_n_args() const override { return n_args; }
    private:
        QString get_arg_description() const;
    };

    /** \brief Represents a description of a command. */
    typedef QString Description;

    /** \brief This is an evil class: it uses variadic templates and lambda
     * closures. */
    class CommandCenter {
        //----types----//
    private:
        enum COMMAND_LIST_ELEMENTS { COM_ALIAS_ARGS, COM_FUNCTION_DESCRIPTION };
        //----members----//
    private:
        std::multimap<
            std::pair<CommandAliasList,QString>,
            std::pair<std::shared_ptr<AbstractCommandFunction>, Description>
            > command_list;
        QString arg_separator = " ";
        QString command_separator = ";";

        //----methods----//
    public:
        CommandCenter() = default;
        virtual ~CommandCenter() = default;
        template<class Func>
            void add_command(const CommandAliasList&, const Func&, const Description&);
        QString execute(QString command_string) const;
        std::vector<QString> get_help(int space = 4) const;
        QString get_help_string(int space = 4) const;
        QString add_space(int from, int to) const;
    };

    template<class Func>
    CommandFunction<Func>::CommandFunction(Func f):
    func(f)
    {
        arg_description = get_arg_description();
    }

    template<class Func>
        ReturnType CommandFunction<Func>::execute(const QStringList& args) const {
        using namespace function_signature;
        int n_got = args.size();
        if(n_got==n_args) {
            // fill the arguments in an array
            QString args_array[n_args];
            for(int arg_idx=0; arg_idx<n_args; ++arg_idx) {
                args_array[arg_idx] = args[arg_idx];
            }
            // map to QString tuple
            N_tuple<n_args,QString> args_tuple;
            args_tuple = array_to_tuple<decltype(args_array),decltype(args_tuple)>(args_array);
            // map to correct types
            arg_type_tuple_type<func_t> types_tuple;
            N_tuple<n_args,bool> ok_tuple;
            auto result_tuple = zip_tuples(ok_tuple,types_tuple);
            map_tuples<decltype(args_tuple),decltype(result_tuple),Mapper>(args_tuple,result_tuple);
            ok_tuple = unzip_tuples_first(result_tuple);
            types_tuple = unzip_tuples_second(result_tuple);
            // check if all args could be mapped
            if(!all_elements_true(ok_tuple)) {
                QString what;
                // get array
                bool ok_arr[n_args];
                tuple_to_array<decltype(ok_tuple),decltype(ok_arr)>(ok_tuple,ok_arr);
                // get array of type descriptors
                N_tuple<n_args,QString> type_descriptor_tuple;
                map_tuples<decltype(types_tuple),decltype(type_descriptor_tuple),TypeDescriptor>(types_tuple,type_descriptor_tuple);
                QString type_descriptor_arr[n_args];
                tuple_to_array<decltype(type_descriptor_tuple),decltype(type_descriptor_arr)>(type_descriptor_tuple,type_descriptor_arr);
                // build return string
                int not_ok_counter = 0;
                bool first = true;
                for(int arg_idx = 0; arg_idx<n_args; ++arg_idx) {
                    if(!ok_arr[arg_idx]) {
                        ++not_ok_counter;
                        if(first) {
                            first = false;
                        } else {
                            what += ", ";
                        }
                        what += QString("%1 (%2 '%3')").arg(arg_idx+1).arg(type_descriptor_arr[arg_idx]).arg(args_array[arg_idx]);
                    }
                }
                if(not_ok_counter>1) {
                    what = "Arguments " + what;
                } else {
                    what = "Argument " + what;
                }
                what += " could not be interpreted";
                return {false,what};
            }
            // bind tuple as function args
            auto func_bound = bind_tuple_as_args<func_t,decltype(types_tuple)>(func,types_tuple);
            // execute function
            return func_bound();
        } else {
            return wrong_number_of_parameters(n_got,n_args);
        }
    }

    template<class Func>
        QString CommandFunction<Func>::get_arg_description() const {
        using namespace function_signature;
        QString ret;
        arg_type_tuple_type<Func> types_tuple;
        constexpr int n_args = std::tuple_size<decltype(types_tuple)>::value;
        N_tuple<n_args,QString> type_descriptor_tuple;
        map_tuples<decltype(types_tuple),decltype(type_descriptor_tuple),TypeDescriptor>(types_tuple,type_descriptor_tuple);
        QString type_descriptor_arr[n_args];
        tuple_to_array<decltype(type_descriptor_tuple),decltype(type_descriptor_arr)>(type_descriptor_tuple,type_descriptor_arr);
        bool first = true;
        for(QString s : type_descriptor_arr) {
            if(first) {
                first = false;
            } else {
                ret += " ";
            }
            ret += s;
        }
        return ret;
    }

    template<class Func>
        void CommandCenter::add_command(const CommandAliasList& com,
                                        const Func& func,
                                        const Description& des) {
        using namespace function_signature;
        static_assert(std::is_same<
                      get_return_type<Func>,
                      ReturnType
                      >::value,
                      "Function must return ReturnType (i.e. std::tuple<bool,QString>)"
            );
        auto f_ptr = std::shared_ptr<AbstractCommandFunction>(new CommandFunction<Func>(func));
        // search for duplicats
        for(auto c : command_list) {
            auto alias_and_args = std::get<COM_ALIAS_ARGS>(c);
            if(alias_and_args.first.has_common_element(com) && alias_and_args.second==f_ptr->arg_description) {
                DEBUG_WARNING("A command with same name and same signature already exists [" << (QString)com << " (" << f_ptr->arg_description << ")]");
                return;
            }
        }
        // add command
        command_list.emplace(
            std::make_pair(com,f_ptr->arg_description),
            std::make_pair(f_ptr, des)
            );
    }

}

#include "debug_exclude.h"

#endif /* COMMANDER_H_ */
