#ifndef PARSER_H
#define PARSER_H

#include <QString>
#include <QTextEdit>

#include <set>
#include <list>
#include <memory> // std::shared_ptr

class Parser
{
public:

    /** Key-Value-Graph structure. Value are stored as strings.*/
    struct Graph {
        /** An Node within a key-value-graph. */
        struct Node {
            std::set<QString> keys;
            int keys_start = -1;
            int keys_end = -1;
            std::set<QString> parents;
            int parents_start = -1;
            int parents_end = -1;
            QString value;
            int value_start = -1;
            int value_end = -1;
            std::shared_ptr<Graph> sub_graph = std::shared_ptr<Graph>(new Graph);
            enum VALUE_TYPE { NONE, BOOL, STRING, FILE, DOUBLE, ARRAY, LIST, SPECIAL, KVG } value_type = NONE;
        };
        std::list<Node> items;
        std::set<QString> keys;
        QString dot();
    };

    /** Iterator for QString that tracks the position. */
    class PosIt {
    public:
        /** Use given iterator and start from given position. */
        PosIt(QString::ConstIterator i, int p): it(i), position(p) {}
        /** Iterate through given string. */
        PosIt(const QString & s): PosIt(s.begin(),0) {}
        /** Use iterator and position from other PosIt object. */
        PosIt(const PosIt & other): PosIt(other.it, other.position) {}
        /** Move forward. */
        PosIt operator++() {++it; ++position; return *this;}
        /** Move backward. */
        PosIt operator--() {--it; --position; return *this;}
        /** Dereference */
        QChar operator*() { return *it; }
        /** Member access. */
        const QChar * operator->() { return it; }
        /** Equality. */
        bool operator==(const QString::ConstIterator & other) { return it==other; }
        /** Inequality. */
        bool operator!=(const QString::ConstIterator & other) { return it!=other; }
        /** Get position. */
        int pos() const { return position; }
    private:
        QString::ConstIterator it;
        int position;
    };

    /** Only static functions in this class. */
    Parser() = delete;

    /** Parse given input as key-value-graph and return syntax-highlighted
    output. Highlighting is done via HTML tags. */
    static Graph parse_graph(const QString & input, QString & output);

    /** Parse given input as key-value-graph. If parsing is not done on first
    level (e.g. when a key-value-graph was given as value within another
    key-value-graph on a higher level) the function returns on a closing '}'.
    Otherwise (first_level=true) parse complete input. A closing '}' would
    result in a syntax error then. */
    static void parse_graph(const QString & input, QString & output, PosIt & in_it, Graph & kvg, bool first_level);

    /** Parse a comment. Terminates on new-lines.*/
    static void parse_comment(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse an error. Terminates on comments and new-lines*/
    static void parse_error(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse a key. */
    static void parse_key(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse a parent. */
    static void parse_parent(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse a normal value. */
    static void parse_value(const QString & input, QString & output, PosIt & in_it, Graph & kvg);

    /** Parse a string value enclosed in "...". */
    static void parse_string_value(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse a file value enclosed in '...'. */
    static void parse_file_value(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse a double. */
    static void parse_double_value(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse an array value enclosed in [...]. */
    static void parse_array_value(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse a list value enclosed in (...). */
    static void parse_list_value(const QString & input, QString & output, PosIt & in_it, Graph & kvg);
    /** Parse a special value enclosed in <...>. */
    static void parse_special_value(const QString & input, QString & output, PosIt & in_it, Graph & kvg);

    /** Retruns whether char exactly matches the regular expression. @param c char to be matched @param r string interpreted as reg-exp */
    static bool is(const QChar & c, const QString r) { return QRegExp(r).exactMatch(c); }
    /** Format string so it is displayed correctly as HTML. Escapes special characters, replaces spaces with &nbsp;. */
    static void toHtml(QString & s);
    /** Returns a string containinc c in HTML compatible form. Escapes special characters, replaces spaces with &nbsp;. */
    static QString toHtml(QChar c);
    /** Fills missing position values (-1, -1). */
    static void fill_missing_positions(Graph & kvg);
};

#endif // PARSER_H
