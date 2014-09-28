#ifndef PARSER_H
#define PARSER_H

#include <QString>
#include <QTextEdit>

class Parser
{
public:
    Parser();

    /** Parse given input as key-value-graph and return syntax-highlighted
    output. Highlighting is done via HTML tags. */
    static void parse_graph(const QString & input, QString & output);

    /** Parse given input as key-value-graph. If parsing is not done on first
    level (e.g. when a key-value-graph was given as value within another
    key-value-graph on a higher level) the function returns on a closing '}'.
    Otherwise (first_level=true) parse complete input. A closing '}' would
    result in a syntax error then. */
    static void parse_graph(const QString & input, QString & output, QString::const_iterator & in_it, bool first_level);

    /** Parse a comment. Terminates on new-lines.*/
    static void parse_comment(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse an error. Terminates on comments and new-lines*/
    static void parse_error(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse a key. */
    static void parse_key(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse a parent. */
    static void parse_parent(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse a normal value. */
    static void parse_value(const QString & input, QString & output, QString::const_iterator & in_it);

    /** Parse a string value enclosed in "...". */
    static void parse_string_value(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse a file value enclosed in '...'. */
    static void parse_file_value(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse a double. */
    static void parse_double_value(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse an array value enclosed in [...]. */
    static void parse_array_value(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse a list value enclosed in (...). */
    static void parse_list_value(const QString & input, QString & output, QString::const_iterator & in_it);
    /** Parse a special value enclosed in <...>. */
    static void parse_special_value(const QString & input, QString & output, QString::const_iterator & in_it);

    /** Retruns whether char exactly matches the regular expression. @param c char to be matched @param r string interpreted as reg-exp */
    static bool is(const QChar & c, const QString r) { return QRegExp(r).exactMatch(c); }
    /** Format string so it is displayed correctly as HTML. Escapes special characters, replaces spaces with &nbsp;. */
    static void toHtml(QString & s);
    /** Returns a string containinc c in HTML compatible form. Escapes special characters, replaces spaces with &nbsp;. */
    static QString toHtml(QChar c);
};

#endif // PARSER_H
