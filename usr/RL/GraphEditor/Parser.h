#ifndef PARSER_H
#define PARSER_H

#include <tuple>
#include <list>
#include <memory> // std::shared_ptr
#include <functional> // std::function

#include <QString>
#include <QTextEdit>

class Parser
{
    //---types/classes---//
private:
    class Node;
    typedef std::shared_ptr<Node> node_ptr_t;
    typedef std::function<void(QString)> transition_function_t;
    typedef std::tuple<node_ptr_t, bool> transition_result_t;
    typedef std::tuple<QString, transition_function_t, transition_result_t> transition_t;
    typedef std::list<transition_t> transition_list_t;
    typedef std::list<node_ptr_t> node_list_t;

    class Node {
        //---members---//
    public:
        const QString name;
    private:
        transition_list_t transitions;
        transition_function_t error_function;
        transition_result_t error_result;
        //---methods---//
    public:
        Node(QString n, transition_function_t error_func = [](QString){}, transition_result_t error_res = transition_result_t(nullptr, true));
        ~Node() = default;
        virtual void add_transition(QString, transition_function_t, node_ptr_t, bool);
        virtual transition_result_t transition(QString);

    };

    //---members---//
private:
    QString output;
    node_ptr_t start_node;
    node_ptr_t error_node;
    QTextEdit * console = nullptr;
    QString toHTML(QString s) const;

    //---methods---//
public:
    Parser();
    virtual QString get_output() const { return output; }
    virtual bool parse_input(QString input);
    virtual void set_console(QTextEdit * c) { console = c; }

};

#endif // PARSER_H
