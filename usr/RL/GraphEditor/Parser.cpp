#include "Parser.h"

#include "util.h"

//static const QString success_string = R"(<span style="color:#aaa; background-color:#eee">&nbsp;</span>&nbsp;)";
//static const QString error_string =   R"(<span style="color:#a00; background-color:#eee">></span>&nbsp;)";

static const QString neutral_color =  R"(<span style="color:#000;">)";
static const QString error_color =    R"(<span style="color:#a00;">)";
static const QString comment_color =  R"(<span style="color:#a95;">)";
static const QString key_color =      R"(<span style="color:#070;">)";
static const QString parent_color =   R"(<span style="color:#0af;">)";
static const QString par_sep_color =  R"(<span style="color:#00f;font-weight:bold;">)";
static const QString value_color =    R"(<span style="color:#f0f;">)";
static const QString val_sep_color =  R"(<span style="color:#90f;font-weight:bold;">)";
static const QString close_span =     R"(</span>)";

static const QString key_chars =      R"([a-zA-Z0-9_])";
static const QString parent_chars =   R"([a-zA-Z0-9_])";
static const QString value_chars =    R"([a-zA-Z0-9_.])";

Parser::Parser()
{
}

void Parser::parse_graph(const QString &input, QString &output)
{
    auto in_it = input.begin();
    parse_graph(input, output, in_it, true);
}

void Parser::parse_graph(const QString &input, QString &output, QString::const_iterator &in_it, bool first_level)
{
    // what comes next?
    enum STATE { START, KEYS, PARENTS, VALUE, SIMPLE_VALUE, GRAPH_VALUE} state = START;

    // iterate through input (hand over to sub-parsers if required)
    while(in_it!=input.end()) {
        // state independent
        QChar c = *in_it;
        if(is(c," ")) { // append space (as HTML)
            DEBUG_OUT("white space");
            output += "&nbsp;";
            ++in_it;
        } else if(c=='\n') { // append new-line (as HTML)
            DEBUG_OUT("new line");
            //DEBUG_OUT("");
            output += "<br>";
            ++in_it;
        } else if(is(c,"#")) { // parse comment
            parse_comment(input, output, in_it);
            state = START;
        } else {
            // state dependent
            switch (state) {
            case START:
                if(is(c,key_chars)) {
                    // keys
                    parse_key(input, output, in_it);
                    state = KEYS;
                } else if(is(c,R"(\()")) {
                    // no keys but parents
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = PARENTS;
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
                    parse_error(input, output, in_it);
                    state = START;
                }
                break;
            case KEYS:
                // expecting more keys
                if(is(c,key_chars)) {
                    parse_key(input, output, in_it);
                } else if(is(c,R"(\()")) {
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = PARENTS;
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
                    parse_error(input, output, in_it);
                    state = START;
                }
                break;
            case PARENTS:
                // expecting parent(s)
                if(is(c,parent_chars)) {
                    parse_parent(input, output, in_it);
                } else if(is(c,",")) {
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                } else if(is(c,R"(\))")) {
                    output += par_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = VALUE;
                } else {
                    parse_error(input, output, in_it);
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
            case SIMPLE_VALUE:
                // expecting value
                if(c=='"') {
                    // explicit string value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_string_value(input, output, in_it);
                    if(in_it!=input.end() && *in_it=='"') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                    } else {
                        parse_error(input, output, in_it);
                    }
                    state = START;
                } else if(c=='\'') {
                    // file value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_file_value(input, output, in_it);
                    if(in_it!=input.end() && *in_it=='\'') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                    } else {
                        parse_error(input, output, in_it);
                    }
                    state = START;
                } else if(c=='[') {
                    // array value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_array_value(input, output, in_it);
                    if(in_it!=input.end() && *in_it==']') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                    } else {
                        parse_error(input, output, in_it);
                    }
                    state = START;
                } else if(c=='(') {
                    // list value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_list_value(input, output, in_it);
                    if(in_it!=input.end() && *in_it==')') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                    } else {
                        parse_error(input, output, in_it);
                    }
                    state = START;
                } else if(c=='<') {
                    // special value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    parse_special_value(input, output, in_it);
                    if(in_it!=input.end() && *in_it=='>') {
                        output += val_sep_color + toHtml(*in_it) + close_span;
                        ++in_it;
                    } else {
                        parse_error(input, output, in_it);
                    }
                    state = START;
                } else if(is(c,"[+-.0-9]")) {
                    // double value
                    parse_double_value(input, output, in_it);
                    state = START;
                } else if(is(c,value_chars)) {
                    // normal value (try only if nothing else matches)
                    parse_value(input, output, in_it);
                    state = START;
                } else {
                    parse_error(input, output, in_it);
                    state = START;
                }
                break;
            case GRAPH_VALUE:
                if(c=='}') {
                    // closing after graph value
                    output += val_sep_color + toHtml(c) + close_span;
                    ++in_it;
                    state = START;
                } else {
                    parse_graph(input, output, in_it, false);
                }
                break;
            }
        }
    }
}

void Parser::parse_comment(const QString &input, QString &output, QString::const_iterator &in_it)
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

void Parser::parse_error(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString error;
    while(in_it!=input.end() && *in_it!='\n' && *in_it!='#') {
        error += *in_it;
        ++in_it;
    }
    ERROR(QString("parse error: %1").arg(error));
    toHtml(error);
    output += error_color + error + close_span;
}

void Parser::parse_key(const QString &input, QString &output, QString::const_iterator &in_it)
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

    // format as HTML
    toHtml(key);
    if(special) {
        output += key_color + "<b>" + key + "</b>" + close_span;
    } else {
        output += key_color + key + close_span;
    }
}

void Parser::parse_parent(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString parent;
    while(in_it!=input.end() && is(*in_it,parent_chars)) {
        parent += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse parent: %1").arg(parent));
    toHtml(parent);
    output += parent_color + parent + close_span;
}

void Parser::parse_value(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString value;
    while(in_it!=input.end() && is(*in_it,value_chars)) {
        value += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse value: %1").arg(value));
    toHtml(value);
    output += value_color + value + close_span;
}

void Parser::parse_string_value(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString string;
    while(in_it!=input.end() && is(*in_it,R"([^"])")) {
        string += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse string value: %1").arg(string));
    toHtml(string);
    output += value_color + string + close_span;
}

void Parser::parse_file_value(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString file;
    while(in_it!=input.end() && is(*in_it,"[^']")) {
        file += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse file value: %1").arg(file));
    toHtml(file);
    output += value_color + file + close_span;
}

void Parser::parse_double_value(const QString &input, QString &output, QString::const_iterator &in_it)
{
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
    toHtml(d);
    output += value_color + d + close_span;

    // if error occurred parse rest as error
    if(error) {
        parse_error(input, output, in_it);
    }
}

void Parser::parse_array_value(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString array;
    while(in_it!=input.end() && is(*in_it,"[^]]")) {
        array += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse array value: %1").arg(array));
    toHtml(array);
    output += value_color + array + close_span;
}

void Parser::parse_list_value(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString list;
    while(in_it!=input.end() && is(*in_it,"[^)]")) {
        list += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse list value: %1").arg(list));
    toHtml(list);
    output += value_color + list + close_span;
}

void Parser::parse_special_value(const QString &input, QString &output, QString::const_iterator &in_it)
{
    QString special;
    while(in_it!=input.end() && is(*in_it,"[^>]")) {
        special += *in_it;
        ++in_it;
    }
    DEBUG_OUT(QString("parse special value: %1").arg(special));
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
