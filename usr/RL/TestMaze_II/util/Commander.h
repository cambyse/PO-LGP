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

#include "debug.h"

namespace Commander {

    /** \brief Represents a list of command aliases. */
    class CommandAliasList {
        //----members----//
    private:
        std::vector<QString> commands;
        //----methods----//
    public:
        CommandAliasList(QString com) {
            commands = {com};
        }
        CommandAliasList(std::initializer_list<QString> coms) {
            commands = std::vector<QString>(coms);
        }
        virtual ~CommandAliasList() = default;
        QString get_string() const {
            QString ret;
            bool first = true;
            for(auto s : commands) {
                if(first) {
                    ret += s;
                    first = false;
                } else {
                    ret += " | " + s;
                }
            }
            return ret;
        }
        bool operator==(const QString& other) {
            for(auto c : commands) {
                if(other==c) {
                    return true;
                }
            }
            return false;
        }
    };

    /** \brief Converter from type U to type T. */
    template<class T, class U>
        T type_interpreter(const U&);

    /** \brief Return type of command-functions.
     *
     * First element signals whether the received parameters were ok, second
     * element is an (error) message. */
    typedef std::pair<bool,QString> ReturnType;

    /** \brief Base type for command-functions. */
    class AbstractCommandFunction {
    public:
        ReturnType wrong_number_of_parameters(int,int) const;
        virtual ReturnType execute(const QStringList&) const = 0;
    };

    /** \brief Templatized version to store specific command-functions. */
    template<class Func>
        class CommandFunction : public AbstractCommandFunction {
    public:
        typedef function_signature::make_function_type<Func> func_t;
        func_t func;
        CommandFunction(Func f): func(f) {}
        virtual ReturnType execute(const QStringList& args) const override {
            using namespace function_signature;
            int n_got = args.size();
            int n_need = get_arg_type_indices(func).size();
            if(n_got==n_need) {
                // fill the arguments in an array (size know at compile time)
                static const int n_args = get_n_args<Func>();
                QString args_array[n_args];
                for(int arg_idx=0; arg_idx<n_args; ++arg_idx) {
                    args_array[arg_idx] = args[arg_idx];
                }
                return {true,"Size OK"};
            } else {
                return wrong_number_of_parameters(n_got,n_need);
            }
        }
    };

    /** \brief Represents a description of a command. */
    class Description: public QString {
    public:
        template<class T>
            Description(const T& t): QString(t) {}
        virtual ~Description() = default;
        QString get_string() const {
            return *this;
        }
    };

    /** \brief This is an evil class: it uses variadic templates and lambda
     * closures. */
    class CommandCenter {
        //----members----//
    private:
        std::vector<std::tuple<
            CommandAliasList,
            std::shared_ptr<AbstractCommandFunction>,
            Description
            > > command_list;

        //----methods----//
    public:
        CommandCenter() = default;
        virtual ~CommandCenter() = default;
        template<class Func>
            void add_command(const CommandAliasList& com,
                             const Func& func,
                             const Description& des) {
            using namespace function_signature;
            static_assert(std::is_same<
                          get_return_type<Func>,
                          ReturnType
                          >::value,
                          "Function must return ReturnType (i.e. std::tuple<bool,QString>)"
                );
            command_list.push_back(
                std::make_tuple(com,
                                std::shared_ptr<AbstractCommandFunction>(
                                    new CommandFunction<Func>(func)
                                    ),
                                des)
                );
        }
        /* std::vector<QString> print_commands(int space = 4) const; */
        /* QString add_space(int from, int to) const; */
        QString execute(QString command_string) const;
    };

}

#include "debug_exclude.h"

#endif /* COMMANDER_H_ */
