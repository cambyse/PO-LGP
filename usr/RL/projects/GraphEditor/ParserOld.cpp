#include "ParserOld.h"

#include <QRegExp>

#include "QtUtil.h"
#include <iostream>

#define DEBUG_OUT(x) {console->append(x); std::cout << x << std::endl;}

using std::get;

//◦●▪
static const QString success_string = R"(<span style="color:#aaa; background-color:#eee">&nbsp;</span>&nbsp;)";
static const QString error_string =   R"(<span style="color:#a00; background-color:#eee">></span>&nbsp;)";

static const QString error_color =    R"(<span style="color:#a00;">)";
static const QString comment_color =  R"(<span style="color:#a95;">)";
static const QString key_color =      R"(<span style="color:#070;">)";
static const QString parent_color =   R"(<span style="color:#00f;">)";
static const QString value_color =    R"(<span style="color:#fb0;">)";

bool ParserOld::parse_input(QString input)
{
    output = "";
    // iterate through input
    node_ptr_t current_node = start_node;
    for(auto input_it=input.begin(); input_it!=input.end();) {
        DEBUG_OUT(QString("%1: '%2'").arg(current_node->name).arg(*input_it));
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

QString ParserOld::toHTML(QString s) const
{
    s = s.toHtmlEscaped();
    s.replace(" ", "&nbsp;");
    return s;
}

ParserOld::ParserOld(): error_node(new Node("Error"))
{

    //---declare nodes and some defaults---//

    // some functions
    transition_function_t do_nothing = [this](QString){};
    transition_function_t append_action = [this](QString s){output+=toHTML(s);};
    transition_function_t close_append_action = [this](QString s){output+="</span>"+toHTML(s);};
    transition_function_t comment_begin_action = [this](QString s){output+=comment_color+toHTML(s);};
    transition_function_t comment_internal_action = append_action;
    transition_function_t comment_terminate_action = close_append_action;
    transition_function_t key_begin_action = [this](QString s){output+=key_color+toHTML(s);};
    transition_function_t key_internal_action = append_action;
    transition_function_t key_terminate_action = close_append_action;
    transition_function_t parent_begin_action = [this](QString s){output+=parent_color+toHTML(s);};
    transition_function_t parent_internal_action = append_action;
    transition_function_t parent_terminate_action = close_append_action;
    transition_function_t value_begin_action = [this](QString s){output+=value_color+toHTML(s);};
    transition_function_t value_internal_action = append_action;
    transition_function_t value_terminate_action = close_append_action;
    // error handling
    transition_function_t error_function([this](QString s){output+=error_color;});
    transition_result_t error_result(error_node, false);

    // other nodes
    start_node = node_ptr_t(new Node("Start", error_function, error_result));
    node_ptr_t comment_node(new Node("Comment", error_function, error_result));
    node_ptr_t key_begin_node(new Node("Key-Begin", error_function, error_result));
    node_ptr_t key_internal_node(new Node("Key-Internal", error_function, error_result));
    node_ptr_t key_terminate_node(new Node("Key-Terminate", error_function, error_result));
    node_ptr_t parent_begin_node(new Node("Parent-Begin", error_function, error_result));
    node_ptr_t parent_internal_node(new Node("Parent-Internal", error_function, error_result));
    node_ptr_t parent_terminate_node(new Node("Parent-Terminate", error_function, error_result));
    node_ptr_t valuePAR_begin_node(new Node("ValuePAR-Begin", error_function, error_result));
    node_ptr_t valuePAR_internal_node(new Node("ValuePAR-Internal", error_function, error_result));
    node_ptr_t valuePAR_terminate_node(new Node("ValuePAR-Terminate", error_function, error_result));
    node_ptr_t valueEQ_begin_node(new Node("ValueEQ-Begin", error_function, error_result));
    node_ptr_t valueEQ_internal_node(new Node("ValueEQ-Internal", error_function, error_result));
    node_ptr_t valueEQ_terminate_node(new Node("ValueEQ-Terminate", error_function, error_result));
    node_ptr_t end_node(new Node("End"));

    //---define transitions---//

    // handle error situation (just append)
    error_node->add_transition(".", append_action, error_node, true);

    // start node
    start_node->add_transition(R"(\s)", append_action, start_node, true);
    start_node->add_transition("", do_nothing, end_node, true);
    start_node->add_transition("#", comment_begin_action, comment_node, false);
    start_node->add_transition("[a-zA-Z0-9_({=]", do_nothing, key_begin_node, false);

    // comments
    comment_node->add_transition(".", comment_internal_action, comment_node, true);
    comment_node->add_transition("", comment_terminate_action, end_node, true);

    // keys
    key_begin_node->add_transition("[a-zA-Z0-9_]", key_begin_action, key_internal_node, true);
    key_begin_node->add_transition(R"(\s)", append_action, key_begin_node, true);
    key_begin_node->add_transition("[#({=]?", do_nothing, key_terminate_node, false);
    key_internal_node->add_transition("[a-zA-Z0-9_]", key_internal_action, key_internal_node, true);
    key_internal_node->add_transition(R"(\s)", append_action, key_begin_node, true);
    key_internal_node->add_transition("[#({=]?", do_nothing, key_terminate_node, false);
    key_terminate_node->add_transition("", do_nothing, end_node, false);
    key_terminate_node->add_transition("#", comment_begin_action, comment_node, false);
    key_terminate_node->add_transition(R"(\()", parent_begin_action, parent_begin_node, true);

    // parents
    parent_begin_node->add_transition("[a-zA-Z0-9_]", parent_begin_action, parent_internal_node, true);
    parent_begin_node->add_transition(R"(\s)", append_action, parent_begin_node, true);
    parent_begin_node->add_transition(R"(\))", parent_terminate_action, parent_terminate_node, true);
    parent_internal_node->add_transition("[a-zA-Z0-9_]", parent_internal_action, parent_internal_node, true);
    parent_internal_node->add_transition(R"(\s)", append_action, parent_begin_node, true);
    parent_internal_node->add_transition(R"(\))", parent_terminate_action, parent_terminate_node, true);
    parent_terminate_node->add_transition("", do_nothing, end_node, false);
    parent_terminate_node->add_transition("#", comment_begin_action, comment_node, false);
    parent_terminate_node->add_transition("=", value_begin_action, valueEQ_begin_node, true);
    parent_terminate_node->add_transition("{", value_begin_action, valueEQ_begin_node, true);
    parent_terminate_node->add_transition(R"(\s)", append_action, parent_terminate_node, true);
}

ParserOld::Node::Node(QString n, transition_function_t error_func, transition_result_t error_res): name(n), error_function(error_func)
{
    if(get<0>(error_res)==nullptr) {
        get<0>(error_res) = node_ptr_t(this);
    }
    error_result = error_res;
}

void ParserOld::Node::add_transition(QString input, transition_function_t trans_func, node_ptr_t node, bool forward)
{
    transitions.push_back(transition_t(input, trans_func, transition_result_t(node, forward)));
}

ParserOld::transition_result_t ParserOld::Node::transition(QString s)
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
