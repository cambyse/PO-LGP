#include "Parser.h"

#include "util.h"

#include <map>
#include <vector>
#include <functional>

using std::vector;
using std::multimap;
using std::map;
using std::list;
using std::shared_ptr;

static const QString error_bg_color = "#fcc";

static const QString neutral_color =  R"(<span style="color:#000;">)";
static const QString error_color =    R"(<span style="color:#a00;background-color:)"+error_bg_color+R"(;">)";
static const QString comment_color =  R"(<span style="color:#a95;">)";
static const QString key_color =      R"(<span style="color:#070;">)";
static const QString parent_color =   R"(<span style="color:#0af;">)";
static const QString par_sep_color =  R"(<span style="color:#00f;font-weight:bold;">)";
static const QString value_color =    R"(<span style="color:#f0f;">)";
static const QString val_sep_color =  R"(<span style="color:#90f;font-weight:bold;">)";
static const QString close_span =     R"(</span>)";

static const QString begin_error_bg = R"(<span style="background-color:)"+error_bg_color+R"(;">)";

static const QString premature_end =  R"(<premature end>)";

static const QString begin_line = R"(<p style="margin:0px;">)";
static const QString end_line   = "</p>\n";

static const QString key_chars =      R"([a-zA-Z0-9_])";
static const QString parent_chars =   R"([a-zA-Z0-9_])";
static const QString value_chars =    R"([a-zA-Z0-9_.])";

Parser::KeyValueGraph Parser::parse_graph(const QString &input, QString &output)
{
    // remove errors from end of input
    QString clean_input = input;
    clean_input.remove(QRegExp("("+premature_end+"\\s*)+$"));
    // hand over to parser
    PosIt in_it(clean_input);
    KeyValueGraph kvg;
    parse_graph(clean_input, output, in_it, kvg, true);
    return kvg;
}

void Parser::fill_missing_positions(KeyValueGraph & kvg)
{
    for(auto& item : kvg.items) {
        // no keys
        if(item.keys_start==-1 && item.keys_end==-1) {
            item.keys_start = item.parents_start;
            item.keys_end = item.parents_start;
        }
        // no parents
        if(item.parents_start==-1 && item.parents_end==-1) {
            item.parents_start = item.keys_end;
            item.parents_end = item.keys_end;
        }
        // no values
        if(item.value_start==-1 && item.value_end==-1) {
            item.value_start = item.parents_end;
            item.value_end = item.parents_end;
        }
        // call recursively
        if(item.value_type==KeyValueGraph::Item::KVG) {
            fill_missing_positions(*(item.sub_graph));
        }
    }
}

void Parser::parse_graph(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg, bool first_level)
{
    // what comes next?
    enum STATE { START, KEYS, PARENTS, VALUE, SIMPLE_VALUE, GRAPH_VALUE} state = START;

    // iterate through input (hand over to sub-parsers if required)
    while(in_it!=input.end()) {
        // state independent
        QChar c = *in_it;
        if(is(c,R"(\s)") && c!='\n') { // append space (as HTML)
            DEBUG_OUT("white space");
            output += toHtml(c);
            ++in_it;
        } else if(c=='#' && (state==PARENTS || state==SIMPLE_VALUE || state==GRAPH_VALUE)) { // unexpected termination
            parse_error(input, output, in_it, kvg);
            state = START;
        } else if(c=='\n') { // append new-line (as HTML)
            // new-line also starts new item when list of keys is not
            // terminated
            if(state==KEYS) {
                state = START;
            }
            DEBUG_OUT("new line");
            //DEBUG_OUT("");
            output += end_line + begin_line;
            ++in_it;
        } else if(c=='#') { // parse comment
            parse_comment(input, output, in_it, kvg);
            state = START;
        } else {
            // state dependent
            switch (state) {
            case START:
                if(is(c,key_chars)) {
                    // keys
                    kvg.items.push_back(KeyValueGraph::Item());
                    parse_key(input, output, in_it, kvg);
                    state = KEYS;
                } else if(is(c,R"(\()")) {
                    // no keys but parents
                    kvg.items.push_back(KeyValueGraph::Item());
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = PARENTS;
                    // set start positions
                    kvg.items.back().parents_start = in_it.pos();
                } else if(!first_level && c=='}') {
                    // return on closing '}' when not parsing on first level
                    // (i.e. when graph was given as a value in some higher
                    // level graph)
                    return;
                } else if(c==',') {
                    // optional separator
                    output += neutral_color + toHtml(c) + close_span;
                    ++in_it;
                } else {
                    parse_error(input, output, in_it, kvg);
                    state = START;
                }
                break;
            case KEYS:
                // expecting more keys
                if(is(c,key_chars)) {
                    parse_key(input, output, in_it, kvg);
                } else if(is(c,R"(\()")) {
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = PARENTS;
                    // set start positions
                    kvg.items.back().parents_start = in_it.pos();
                } else if(c=='=' || c=='{') {
                    state = VALUE;
                } else if(c==',') {
                    // optional separator/terminator
                    output += neutral_color + toHtml(c) + close_span;
                    ++in_it;
                    state = START;
                } else if(c=='}') {
                    // return to top level (no error: could be nested graphs)
                    state = START;
                } else {
                    parse_error(input, output, in_it, kvg);
                    state = START;
                }
                break;
            case PARENTS:
                // expecting parent(s)
                if(is(c,parent_chars)) {
                    parse_parent(input, output, in_it, kvg);
                } else if(is(c,",")) {
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                } else if(is(c,R"(\))")) {
                    // set end positions
                    kvg.items.back().parents_end = in_it.pos();
                    // print closing ')'
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = VALUE;
                } else {
                    parse_error(input, output, in_it, kvg);
                    state = START;
                }
                break;
            case VALUE:
                // expecting initializer for value
                if(c=='=') {
                    //  initializer '='
                    output += val_sep_color + c;
                    ++in_it;
                    // combination also allowed: '={'
                    if(in_it!=input.end() && *in_it=='{') {
                        output += *in_it;
                        ++in_it;
                        output += close_span;
                        state = GRAPH_VALUE;
                    } else {
                        output += close_span;
                        state = SIMPLE_VALUE;
                    }
                } else if(c=='{') {
                    // initializer '{'
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = GRAPH_VALUE;
                } else if(c==',') {
                    // no value, go back to start
                    output += neutral_color + toHtml(c) + close_span;
                    ++in_it;
                } else if(c=='}') {
                    // return to top level (no error: could be nested graphs)
                    state = START;
                } else {
                    // no nothing go back to start
                    state = START;
                }
                break;
            case SIMPLE_VALUE: // expecting value
                // set start position
                kvg.items.back().value_start = in_it.pos();
                // decide on value type
                if(c=='"') {
                    // explicit string value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_string_value(input, output, in_it, kvg);
                    if(in_it!=input.end() && *in_it=='"') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                    } else {
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                        // error
                        parse_error(input, output, in_it, kvg);
                    }
                    state = START;
                } else if(c=='\'') {
                    // file value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_file_value(input, output, in_it, kvg);
                    if(in_it!=input.end() && *in_it=='\'') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                    } else {
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                        // error
                        parse_error(input, output, in_it, kvg);
                    }
                    state = START;
                } else if(c=='[') {
                    // array value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_array_value(input, output, in_it, kvg);
                    if(in_it!=input.end() && *in_it==']') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                    } else {
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                        // error
                        parse_error(input, output, in_it, kvg);
                    }
                    state = START;
                } else if(c=='(') {
                    // list value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_list_value(input, output, in_it, kvg);
                    if(in_it!=input.end() && *in_it==')') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                    } else {
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                        // error
                        parse_error(input, output, in_it, kvg);
                    }
                    state = START;
                } else if(c=='<') {
                    // special value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_special_value(input, output, in_it, kvg);
                    if(in_it!=input.end() && *in_it=='>') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                    } else {
                        // set end position
                        kvg.items.back().value_end = in_it.pos();
                        // error
                        parse_error(input, output, in_it, kvg);
                    }
                    state = START;
                } else if(is(c,"[+-.0-9]")) {
                    // double value
                    parse_double_value(input, output, in_it, kvg);
                    state = START;
                    // set end position
                    kvg.items.back().value_end = in_it.pos();
                } else if(is(c,value_chars)) {
                    // normal value (try only if nothing else matches)
                    parse_value(input, output, in_it, kvg);
                    state = START;
                    // set end position
                    kvg.items.back().value_end = in_it.pos();
                } else {
                    // set end position
                    kvg.items.back().value_end = in_it.pos();
                    // error
                    parse_error(input, output, in_it, kvg);
                    state = START;
                }
                break;
            case GRAPH_VALUE:
                // set start position
                kvg.items.back().value_start = in_it.pos();
                // parse or close
                if(c=='}') {
                    // closing after graph value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = START;
                    // set end position
                    kvg.items.back().value_end = in_it.pos();
                } else {
                    KeyValueGraph sub_graph;
                    parse_graph(input, output, in_it, sub_graph, false);
                    // set value in key-value-graph
                    auto& item = kvg.items.back();
                    if(item.value_type!=KeyValueGraph::Item::NONE) {
                        ERROR("Overwriting value");
                    }
                    item.value = "";
                    *(item.sub_graph) = sub_graph;
                    item.value_type = KeyValueGraph::Item::KVG;
                }
                break;
            }
        }
    }

    // set missing values to default (true)
    list<KeyValueGraph*> kvg_list({&kvg});
    while(!kvg_list.empty()) {
        for(auto& item : kvg_list.front()->items) {
            switch(item.value_type) {
            case KeyValueGraph::Item::NONE:
                item.value = "true";
                item.value_type = KeyValueGraph::Item::BOOL;
                break;
            case KeyValueGraph::Item::KVG:
                kvg_list.push_back(item.sub_graph.get());
                break;
            default:
                break;
            }
        }
        kvg_list.pop_front();
    }

    // set positions if something is missing
    fill_missing_positions(kvg);

    // unexpected termination of whole graph
    if(state==PARENTS || state==SIMPLE_VALUE || state==GRAPH_VALUE) {
        parse_error(input, output, in_it, kvg);
    }

    // on top-level
    if(first_level) {
        // properly start first line and end last line
        output = begin_line + output + end_line;
        // add <code> tags
        output = "<code>\n" + output + "</code>";
    }

    // convert empty paragraphs to line breaks within preceeding paragraph
    {
        QString new_out = output;
        while(output!=new_out.replace(end_line+begin_line+end_line,"<br>"+end_line)) {
            output = new_out;
        }
        // does not work for empty paragraphs at beginning
        while(output!=new_out.replace(begin_line+end_line+begin_line,begin_line+"<br>")) {
            output = new_out;
        }
    }

    // circumvent Qt display bug
    {
        // from one or more &nbsp; remove the first if it is preceeded
        // by a </span> and the last &nbsp; is followed by a <br>
        QRegExp workaround("</span>(&nbsp;)+<br>");
        while(workaround.indexIn(output)>=0) {
            QString match = workaround.capturedTexts()[0];
            QString match_fixed = match;
            match_fixed.replace(QRegExp("</span>&nbsp;"),"</span> ");
            output.replace(match,match_fixed);
        }
    }
}

void Parser::parse_comment(const QString &input, QString &output, PosIt &in_it, KeyValueGraph &)
{
    QString com;
    while(in_it!=input.end() && *in_it!='\n') {
        com += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse comment: %1").arg(com));
    toHtml(com);
    output += comment_color + com + close_span;
}

void Parser::parse_error(const QString &input, QString &output, PosIt &in_it, KeyValueGraph &)
{
    QString error;
    bool parsed_something = false;
    while(in_it!=input.end() && *in_it!='\n') {
        parsed_something = true;
        error += *in_it;
        ++in_it;
    }
    ERROR(QString("parse error: %1").arg(error));
    // add space to visualize error if nothing was parsed
    if(!parsed_something) {
        error += premature_end;
    }
    // convert special chars and spaces to HTML
    toHtml(error);
    output += error_color + error + close_span;
}

void Parser::parse_key(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parse key
    QString key;
    while(in_it!=input.end() && is(*in_it,key_chars)) {
        key += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse key: %1").arg(key));

    // check for special keys (Include, Merge)
    bool special = false;
    if(key=="Include" || key=="Merge") {
        special = true;
    }
    // set positions
    auto& item = kvg.items.back();
    if(item.keys.size()==0) {
        item.keys_start = in_it.pos() - key.size();
    }
    item.keys_end = in_it.pos();
    // add key to item
    item.keys.insert(key);
    // check if key was defined before or insert
    bool already_defined = false;
//#define UNIQUE_KEYS
#ifdef UNIQUE_KEYS
    auto& keys = kvg.keys;
    if(keys.find(key)==keys.end()) {
        keys.insert(key);
    } else {
        already_defined = true;
    }
#undef UNIQUE_KEYS
#else
    kvg.keys.insert(key);
#endif

    // format as HTML
    toHtml(key);
    if(special) {
        output += key_color + "<b>" + key + "</b>" + close_span;
    } else {
        if(already_defined) {
            output += key_color + begin_error_bg + key + close_span + close_span;
        } else {
            output += key_color + key + close_span;
        }
    }
}

void Parser::parse_parent(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parse parent
    QString parent;
    while(in_it!=input.end() && is(*in_it,parent_chars)) {
        parent += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse parent: %1").arg(parent));

    // check if parent exists or add
    auto& item = kvg.items.back();
    bool parent_already_defined = false;
    if(item.parents.find(parent)==item.parents.end()) {
        item.parents.insert(parent);
    } else {
        parent_already_defined = true;
    }
    // check if key exists
    bool key_exists = false;
    auto& keys = kvg.keys;
    if(keys.find(parent)==keys.end()) {
        keys.insert(parent);
    } else {
        key_exists = true;
    }

    // format as html
    toHtml(parent);
    if(parent_already_defined || !key_exists) {
        output += parent_color + begin_error_bg + parent + close_span + close_span;
    } else {
        output += parent_color + parent + close_span;
    }
}

void Parser::parse_value(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parse value
    QString value;
    while(in_it!=input.end() && is(*in_it,value_chars)) {
        value += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse value: %1").arg(value));

    // set value in key-value-graph
    auto& item = kvg.items.back();
    if(item.value_type!=KeyValueGraph::Item::NONE) {
        ERROR("Overwriting value");
    }
    item.value = value;
    item.value_type = KeyValueGraph::Item::STRING;

    // convert to bool if necessary
    if(value=="true" || value=="false") {
        item.value_type = KeyValueGraph::Item::BOOL;
    }

    // format as HTML
    toHtml(value);
    output += value_color + value + close_span;
}

void Parser::parse_string_value(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parse value
    QString string;
    while(in_it!=input.end() && is(*in_it,R"([^"])")) {
        string += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse string value: %1").arg(string));

    // set value in key-value-graph
    auto& item = kvg.items.back();
    if(item.value_type!=KeyValueGraph::Item::NONE) {
        ERROR("Overwriting value");
    }
    item.value = string;
    item.value_type = KeyValueGraph::Item::STRING;

    // format as HTML
    toHtml(string);
    output += value_color + string + close_span;
}

void Parser::parse_file_value(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parsing value
    QString file;
    while(in_it!=input.end() && is(*in_it,"[^']")) {
        file += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse file value: %1").arg(file));

    // set value in key-value-graph
    auto& item = kvg.items.back();
    if(item.value_type!=KeyValueGraph::Item::NONE) {
        ERROR("Overwriting value");
    }
    item.value = file;
    item.value_type = KeyValueGraph::Item::FILE;

    // format as HTML
    toHtml(file);
    output += value_color + file + close_span;
}

void Parser::parse_double_value(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parsing value
    QString d;
    bool error = false;
    bool sign_allowed = true;
    bool point_allowed = true;
    while(in_it!=input.end() && is(*in_it,"[+-.0-9]")) {
        if(*in_it=='+' || *in_it=='-') {
            if(sign_allowed) {
                sign_allowed = false;
            } else {
                error = true;
                break;
            }
        } else if(*in_it=='.') {
            if(point_allowed) {
                point_allowed = false;
                sign_allowed = false;
            } else {
                error = true;
                break;
            }
        }
        d += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse double value: %1").arg(d));

    // set value in key-value-graph
    auto& item = kvg.items.back();
    if(item.value_type!=KeyValueGraph::Item::NONE) {
        ERROR("Overwriting value");
    }
    item.value = d;
    item.value_type = KeyValueGraph::Item::DOUBLE;

    // format as HTML
    toHtml(d);
    output += value_color + d + close_span;

    // if error occurred parse rest as error
    if(error) {
        parse_error(input, output, in_it, kvg);
    }
}

void Parser::parse_array_value(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parse value
    QString array;
    while(in_it!=input.end() && is(*in_it,"[^]]")) {
        array += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse array value: %1").arg(array));

    // set value in key-value-graph
    auto& item = kvg.items.back();
    if(item.value_type!=KeyValueGraph::Item::NONE) {
        ERROR("Overwriting value");
    }
    item.value = array;
    item.value_type = KeyValueGraph::Item::ARRAY;

    // format as HTML
    toHtml(array);
    output += value_color + array + close_span;
}

void Parser::parse_list_value(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parse value
    QString list;
    while(in_it!=input.end() && is(*in_it,"[^)]")) {
        list += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse list value: %1").arg(list));

    // set value in key-value-graph
    auto& item = kvg.items.back();
    if(item.value_type!=KeyValueGraph::Item::NONE) {
        ERROR("Overwriting value");
    }
    item.value = list;
    item.value_type = KeyValueGraph::Item::LIST;

    // format as HTML
    toHtml(list);
    output += value_color + list + close_span;
}

void Parser::parse_special_value(const QString &input, QString &output, PosIt &in_it, KeyValueGraph & kvg)
{
    // parse value
    QString special;
    while(in_it!=input.end() && is(*in_it,"[^>]")) {
        special += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse special value: %1").arg(special));

    // set value in key-value-graph
    auto& item = kvg.items.back();
    if(item.value_type!=KeyValueGraph::Item::NONE) {
        ERROR("Overwriting value");
    }
    item.value = special;
    item.value_type = KeyValueGraph::Item::SPECIAL;

    // format as HTML
    toHtml(special);
    output += value_color + special + close_span;
}

void Parser::toHtml(QString & s)
{
    s = s.toHtmlEscaped();
    s.replace(" ", "&nbsp;");
}

QString Parser::toHtml(QChar c)
{
    QString s(c);
    toHtml(s);
    return s;
}


QString Parser::KeyValueGraph::dot()
{
    // to find the identifiers of all items with a given key
    int node_counter = 0;
    int cluster_level = 0;
    vector<int> cluster_level_counts({0});
    bool missing_parents = false;
    QString output;
    std::function<void(const KeyValueGraph * kvg, QString cluster_name)> parse_kvg;
    parse_kvg = [&](const KeyValueGraph * kvg, QString cluster_name) {
        multimap<QString, QString> key_id_map;
        QString indentation = QString("    ").repeated(cluster_level+1);
        bool first_node = true;
        for(Item item : kvg->items) {
            QString key_list;
            bool first_key = true;
            for(QString key : item.keys) {
                if(first_key) {
                    first_key = false;
                    key_list += key;
                } else {
                    key_list += ", " + key;
                }
            }
            QString node_id;
            switch (item.value_type) {
            case Item::KVG:
            {
                QString next_cluster_name = QString("cluster_%1_%2").arg(cluster_level).arg(cluster_level_counts[cluster_level]);
                node_id = next_cluster_name;
                output += indentation + "subgraph " + next_cluster_name + " { label = <<b>" + key_list + "</b>>;\n";
                ++cluster_level_counts[cluster_level];
                ++cluster_level;
                if((int)cluster_level_counts.size()<=cluster_level) {
                    cluster_level_counts.push_back(0);
                }
                parse_kvg(item.sub_graph.get(), next_cluster_name);
                --cluster_level;
                output += indentation + "}\n";
            }
                break;
            default:
            {
                node_id = QString("node%1").arg(node_counter);
                if(first_node) {
                    first_node = false;
                    node_id = cluster_name;
                }
                output += indentation + node_id;
                ++node_counter;
                output += " [label=<";
                if(key_list!="") {
                    output += "<b>" + key_list + "</b><br/>";
                }
                output += "<i>" + item.value + "</i>>];\n";
            }
                break;
            }
            // key-id pairs to map
            for(QString key : item.keys) {
                key_id_map.insert({{key, node_id}});
            }
            // make parents
            for(QString par : item.parents) {
                auto match_iters = key_id_map.equal_range(par);
                if(match_iters.first==match_iters.second) {
                    missing_parents = true;
                    output += indentation + "missing_parents -> " + node_id;
                    if(node_id.startsWith("cluster")) {
                        output += " [lhead=" + node_id + "]";
                    }
                    output += ";\n";
                } else {
                    for(auto id_it=match_iters.first; id_it!=match_iters.second; ++id_it) {
                        QString par_id = id_it->second;
                        output += indentation + par_id + " -> " + node_id;
                        if(par_id.startsWith("cluster") && node_id.startsWith("cluster")) {
                            output += " [ltail=" + par_id + ", lhead=" + node_id + "]";
                        } else if(par_id.startsWith("cluster") && par_id!=cluster_name) {
                            output += " [ltail=" + par_id + "]";
                        } else if(node_id.startsWith("cluster")) {
                            output += " [lhead=" + node_id + "]";
                        }
                        output += ";\n";
                    }
                }
            }
        }
    };
    parse_kvg(this, "node0");
    output = QString("digraph { compound = true;%1\n%2}").arg((missing_parents?" missing_parents [label=<missing parents>, color=red];":"")).arg(output);
//    ERROR(output);
    return output;
}
