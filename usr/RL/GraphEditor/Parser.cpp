#include "Parser.h"

#include <QRegExp>

#include "QtUtil.h"
#include <iostream>

#define DEBUG_OUT(x) {console->append(x); std::cout << x << std::endl;}

using std::get;

//◦●▪
static const QString success_string = R"(<span style="color:#aaa; background-color:#eee">&nbsp;</span>&nbsp;)";
static const QString error_string =   R"(<span style="color:#a00; background-color:#eee">◦</span>&nbsp;)";

static const QString error_color =    R"(<span style="color:#a00;">)";
static const QString comment_color =  R"(<span style="color:#a95;">)";
static const QString key_color =      R"(<span style="color:#070;">)";
static const QString parent_color =   R"(<span style="color:#00f;">)";
static const QString value_color =    R"(<span style="color:#fb0;">)";

bool Parser::parse_input(QString input)
{
    output = "";
    // iterate through input
    node_ptr_t current_node = start_node;
    for(auto input_it=input.begin(); input_it!=input.end();) {
        DEBUG_OUT(current_node->name);
        auto result = current_node->transition(*input_it);
        current_node = get<0>(result);
        if(get<1>(result)) {
            ++input_it;
        }
    }
    // wait until termination
    int max_trans = 1000;
    while(max_trans>0) {
        DEBUG_OUT(current_node->name);
        node_ptr_t next_node = get<0>(current_node->transition(""));
        if(next_node==current_node) {
            break;
        } else {
            current_node = next_node;
        }
        --max_trans;
    }

    // return
    if(max_trans<=0) {
        DEBUG_OUT("FORCED TERMINATION");
        output = error_string+output;
        return false;
    } else if(current_node==error_node) {
        output = error_string+output;
        return false;
    } else {
        output = success_string+output;
        return true;
    }
}

QString Parser::toHTML(QString s) const
{
    s = s.toHtmlEscaped();
    s.replace(" ", "&nbsp;");
    return s;
}

Parser::Parser(): error_node(new Node("Error"))
{

    //---declare nodes and some defaults---//

    // some functions
    transition_function_t append = [this](QString s){output+=toHTML(s);};
    transition_function_t close_append = [this](QString s){output+="</span>"+toHTML(s);};
    transition_function_t comment_begin = [this](QString s){output+=comment_color+toHTML(s);};
    transition_function_t comment_within = append;
    transition_function_t comment_terminate = close_append;
    transition_function_t key_begin = [this](QString s){output+=key_color+toHTML(s);};
    transition_function_t key_within = append;
    transition_function_t key_terminate = close_append;
    transition_function_t parent_begin = [this](QString s){output+=parent_color+toHTML(s);};
    transition_function_t parent_within = append;
    transition_function_t parent_terminate = close_append;
    transition_function_t value_begin = [this](QString s){output+=value_color+toHTML(s);};
    transition_function_t value_within = append;
    transition_function_t value_terminate = close_append;
    // error handling
    transition_function_t error_function([this](QString s){output+=error_color;});
    transition_result_t error_result(error_node, false);

    // other nodes
    start_node = node_ptr_t(new Node("Start", error_function, error_result));
    node_ptr_t comment(new Node("Comment", error_function, error_result));
    node_ptr_t key(new Node("Key", error_function, error_result));
    node_ptr_t key_stop(new Node("Key-Stop", error_function, error_result));
    node_ptr_t parent(new Node("Parent", error_function, error_result));
    node_ptr_t parent_stop(new Node("Parent-Stop", error_function, error_result));
    node_ptr_t valuePAR(new Node("ValuePAR", error_function, error_result));
    node_ptr_t valuePAR_stop(new Node("ValuePAR-Stop", error_function, error_result));
    node_ptr_t valueEQ(new Node("ValueEQ", error_function, error_result));
    node_ptr_t valueEQ_stop(new Node("ValueEQ-Stop", error_function, error_result));
    node_ptr_t end(new Node("End"));

    //---define transitions---//

    // handle error situation (just append)
    error_node->add_transition(".", append, error_node, true);

    // start node
    start_node->add_transition(R"(\s)", append, start_node, true);
    start_node->add_transition("", append, end, true);
    start_node->add_transition("#", comment_begin, comment, true);
    start_node->add_transition("[a-zA-Z0-9]", key_begin, key, true);
    start_node->add_transition(R"(\()", parent_begin, parent, true);
    start_node->add_transition("{", value_begin, valuePAR, true);
    start_node->add_transition("=", value_begin, valueEQ, true);

    // comments
    comment->add_transition(".", comment_within, comment, true);
    comment->add_transition("", comment_terminate, end, true);

    // keys
    key->add_transition("[a-zA-Z0-9]", key_within, key, true);
    key->add_transition(R"(\s)", key_terminate, key_stop, true);
}

Parser::Node::Node(QString n, transition_function_t error_func, transition_result_t error_res): name(n), error_function(error_func)
{
    if(get<0>(error_res)==nullptr) {
        get<0>(error_res) = node_ptr_t(this);
    }
    error_result = error_res;
}

void Parser::Node::add_transition(QString input, transition_function_t trans_func, node_ptr_t node, bool forward)
{
    transitions.push_back(transition_t(input, trans_func, transition_result_t(node, forward)));
}

Parser::transition_result_t Parser::Node::transition(QString s)
{
    for(auto tran : transitions) {
        if(QRegExp(get<0>(tran)).exactMatch(s)) {
            get<1>(tran)(s);
            return get<2>(tran);
        }
    }
    error_function(s);
    return error_result;
}
