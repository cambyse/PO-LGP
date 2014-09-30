#include "Parser.h"

#include "util.h"

//static const QString success_string = R"(<span style="color:#aaa; background-color:#eee">&nbsp;</span>&nbsp;)";
//static const QString error_string =   R"(<span style="color:#a00; background-color:#eee">></span>&nbsp;)";

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

static const QString key_chars =      R"([a-zA-Z0-9_])";
static const QString parent_chars =   R"([a-zA-Z0-9_])";
static const QString value_chars =    R"([a-zA-Z0-9_.])";

Parser::KeyValueGraph Parser::parse_graph(const QString &input, QString &output)
{
    // remove errors from end of input
    QString clean_input = input;
    QString input_copy = input;
    while(input_copy!=clean_input.remove(QRegExp(premature_end+"$"))) {
        input_copy = clean_input;
    }
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
        if(c==' ') { // append space (as HTML)
            DEBUG_OUT("white space");
            output += "&nbsp;";
            ++in_it;
        } else if(c=='#' && (state==PARENTS || state==SIMPLE_VALUE || state==GRAPH_VALUE)) { // unexpected termination
            parse_error(input, output, in_it, kvg);
            state = START;
        } else if(c=='\n') { // append new-line (as HTML)
            DEBUG_OUT("new line");
            //DEBUG_OUT("");
            output += "<br>";
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
    for(auto& item : kvg.items) {
        if(item.value_type==KeyValueGraph::Item::NONE) {
            item.value = "true";
            item.value_type = KeyValueGraph::Item::BOOL;
        }
    }

    // set positions if something is missing
    fill_missing_positions(kvg);

    // unexpected termination of whole graph
    if(state==PARENTS || state==SIMPLE_VALUE || state==GRAPH_VALUE) {
        parse_error(input, output, in_it, kvg);
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
    return R"(digraph G {compound=true;a -> b;c -> b;subgraph cluster1 {d -> e;}b -> e [lhead=cluster1,];})";
}
