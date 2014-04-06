#ifndef COMMANDER_H_
#define COMMANDER_H_

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
    class CommandList {
        //----members----//
    private:
        std::vector<QString> commands;
        //----methods----//
    public:
        CommandList(QString com) {
            commands = {com};
        }
        CommandList(std::vector<QString> coms) {
            commands = coms;
        }
        virtual ~CommandList() = default;
        QString get_string() const {
            QString ret;
            bool first = true;
            for(auto s : commands) {
                if(first) {
                    ret += s;
                    first = false;
                } else {
                    ret += " / " + s;
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

    /** \brief Type representing alternatives. */
    /* class Or {}; */

    /** \brief Abstract base class for arguments. */
    /* class AbstractArguments { */
    /* public: */
    /*     virtual QString get_string() const = 0; */
    /* }; */

    /** \brief Represent a variable number of arguments. */
    /* template<class ... Args> */
    /*     class Arguments: public AbstractArguments { */
    /*     //----typedefs/classes----// */
    /* public: */
    /*     typedef std::function<void(Args...)> function_t; */
    /* private: */
    /*     class NoType {}; */
    /*     //----members----// */
    /* private: */
    /*     std::vector<std::type_index> type_descriptors; */
    /*     //----methods----// */
    /* public: */
    /*     Arguments() { */
    /*         if(sizeof...(Args)>0) { */
    /*             type_descriptors = get_type_descriptors<Args...>(); */
    /*         } */
    /*     } */
    /*     virtual ~Arguments() = default; */
    /*     virtual QString get_string() const override { */
    /*         QString ret; */
    /*         bool first = true; */
    /*         for(auto t : type_descriptors) { */
    /*             if(first) { */
    /*                 first = false; */
    /*             } else { */
    /*                 ret += " "; */
    /*             } */
    /*             if(t==std::type_index(typeid(int))) { */
    /*                 ret += "<int>"; */
    /*             } else if(t==std::type_index(typeid(double))) { */
    /*                 ret += "<double>"; */
    /*             } else if(t==std::type_index(typeid(float))) { */
    /*                 ret += "<float>"; */
    /*             } else if(t==std::type_index(typeid(bool))) { */
    /*                 ret += "<bool>"; */
    /*             } else if(t==std::type_index(typeid(QString))) { */
    /*                 ret += "<QString>"; */
    /*             } else if(t==std::type_index(typeid(Or))) { */
    /*                 ret += "|"; */
    /*             } else { */
    /*                 ret += "<unknown>"; */
    /*             } */
    /*         } */
    /*         return ret; */
    /*     } */
    /* private: */
    /*     template<typename First = NoType> */
    /*         std::vector<std::type_index> get_type_descriptors() */
    /*         { */
    /*             if(typeid(First)==typeid(NoType)) { */
    /*                 return {}; */
    /*             } else { */
    /*                 return {std::type_index(typeid(First))}; */
    /*             } */
    /*         } */

    /*     template<typename First, typename Second, typename ... More> */
    /*         std::vector<std::type_index> get_type_descriptors() */
    /*     { */
    /*         auto v1 = get_type_descriptors<First>(); */
    /*         auto v2 = get_type_descriptors<Second,More...>(); */
    /*         v1.insert(v1.end(),v2.begin(),v2.end()); */
    /*         return v1; */
    /*     } */
    /* }; */

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

    typedef std::pair<bool,QString> ReturnType;

    template<class ... Args>
        class Function : public std::function<ReturnType(Args...)> {};

    /** \brief This is an evil class: it uses variadic templates and lambda closures. */
    class CommandCenter {
        //----typedefs/classes----//
    public:

        //----members----//
        std::vector<std::tuple<
            CommandList,
            //std::shared_ptr<const AbstractArguments>,
            //std::shared_ptr<const AbstractArguments>,
            std::function<void()>,
            Description
            > > commands;

        //----methods----//
    public:
        CommandCenter() = default;
        virtual ~CommandCenter() = default;
        template<template<class...> class Func, class ... Args>
            void add_command(//const CommandList& com,
                const Func<Args...>& func//,
                             /* const Description& des */) {
            /* commands.push_back( */
            /*     std::make_tuple(com, */
            /*                     std::function<ReturnType(Args...)>(func), */
            /*                     des) */
            /*     ); */
        }
        /* std::vector<QString> print_commands(int space = 4) const; */
        /* QString add_space(int from, int to) const; */
        /* void execute(QString command_name) const; */
    };

}

#include "debug_exclude.h"

#endif /* COMMANDER_H_ */
