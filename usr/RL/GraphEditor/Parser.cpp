#include "Parser.h"

#include <QRegExp>

#define DEBUG_OUT(x) {console->append(x);}
#define OUT(x) {console->append(x);}

using std::get;

void Parser::parse_input(QString input)
{
    output = "";
    output = "<font face=\"monospace\">";
    node_ptr_t current_node = node_list.front();
    for(auto input_it=input.begin(); input_it!=input.end();) {
        DEBUG_OUT(current_node->name);
        auto result = current_node->transition(*input_it);
        current_node = get<0>(result);
        if(get<1>(result)) {
            ++input_it;
        }
    }
    int max_trans = 1000;
    while(current_node!=node_list.back() && max_trans>0) {
        DEBUG_OUT(current_node->name);
        current_node = get<0>(current_node->transition(""));
        --max_trans;
    }
    DEBUG_OUT(current_node->name);
    if(max_trans<=0) {
        DEBUG_OUT("FORCED TERMINATION");
    }
}

QString Parser::toHTML(QString s) const
{
    s = s.toHtmlEscaped();
    s.replace(" ", "&nbsp;");
    return s;
}

Parser::Parser()
{
    //---declare node and some defaults---//

    // some standard actions
    transition_function_t do_nothing = [](QString){};
    transition_function_t append = [this](QString s){output+=toHTML(s);};
    transition_function_t enter_comment = [this](QString s){output+="<font color=\"#a95\">"+toHTML(s);};
    // the terminal node
    node_ptr_t end(new Node("Terminal"));
    // default transition to end-node with error (red color)
    transition_t default_transition("", [this](QString s){output+="<font color=\"#a00\">"+toHTML(s);}, transition_result_t(end, true));
    // other nodes
    node_ptr_t start(new Node("Start", default_transition));
    node_ptr_t comment(new Node("Comment", default_transition));
    node_ptr_t var(new Node("Variable", default_transition));
    node_ptr_t any(new Node("Anything", default_transition));

    //---define transitions---//

    // enter comment
    for(auto node : {start, any})
        node->add_transition("#", enter_comment, comment, true);
    // exit comment
    comment->add_transition("", append, end, true);
    // within comment
    comment->add_transition(".*", append, comment, true);

    // enter anything else
    for(auto node : {start})
        node->add_transition(".*", do_nothing, any, false);
    // exit anythin else
    any->add_transition("", append, end, true);
    // within anything else
    any->add_transition(".*", append, any, true);

    // process what is left at the end
    end->add_transition(".*", append, end, true);

    // store in list so they don't get destroyed
    node_list = node_list_t({start, comment, any, end});
}

Parser::Node::Node(QString n, transition_t default_trans): name(n)
{
    if(get<0>(get<2>(default_trans))==nullptr) {
        get<0>(get<2>(default_trans)) = node_ptr_t(this);
    }
    default_transition = default_trans;
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
    get<1>(default_transition)(s);
    return get<2>(default_transition);
}
