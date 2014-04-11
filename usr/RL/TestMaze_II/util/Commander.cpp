#include "Commander.h"

#include "debug.h"

using std::vector;
using std::get;
using std::max;
using std::shared_ptr;

namespace Commander {

    using namespace function_signature;

    CommandAliasList::CommandAliasList(QString com): commands({com}) { }

    CommandAliasList::CommandAliasList(const char* com): commands({com}) { }

    CommandAliasList::CommandAliasList(std::initializer_list<QString> coms): commands(std::vector<QString>(coms)) { }

    QString CommandAliasList::get_string() const {
        QString ret;
        bool first = true;
        for(auto s : commands) {
            if(first) {
                ret += s;
                first = false;
            } else {
                ret += " | " + s;
            }
        }
        return ret;
    }

    bool CommandAliasList::contains(const QString& other) const {
        for(auto c : commands) {
            if(other==c) {
                return true;
            }
        }
        return false;
    }

    bool CommandAliasList::has_common_element(const CommandAliasList& other) const {
        for(auto c : commands) {
            if(other.contains(c)) {
                return true;
            }
        }
        return false;
    }

    ReturnType AbstractCommandFunction::wrong_number_of_parameters(int got, int need) const {
        return {false, QString("Wrong number of parameters (got %1, need %2)").arg(got).arg(need)};
    }

    vector<QString> CommandCenter::get_help(int space) const {
        // return
        std::vector<QString> ret;
        // get maximum lengths
        int max_com_length = 0;
        int max_arg_length = 0;
        for(auto current_command : command_set) {
            max_com_length = max(max_com_length,get<COM_ALIAS>(current_command).length());
            max_arg_length = max(max_arg_length,get<COM_ARGS>(current_command).length());
        }
        // adjust for header
        QString com_header("COMMAND");
        QString arg_header("ARGUMENTS");
        QString des_header("DESCRIPTION");
        max_com_length = max(max_com_length,com_header.length());
        max_arg_length = max(max_arg_length,arg_header.length());
        // print header
        {
            int running_pos = 0;
            ret.push_back("");
            ret.back() += com_header;
            ret.back() += add_space(running_pos+com_header.length(),running_pos+max_com_length+space);
            running_pos += max_com_length+space;
            ret.back() += arg_header;
            ret.back() += add_space(running_pos+arg_header.length(),running_pos+max_arg_length+space);
            running_pos += max_arg_length+space;
            ret.back() += des_header;
        }
        // print commands
        QString old_com_str("");
        QStringList old_multi_com_list;
        for(auto current_command : command_set) {
            // add new help entry
            ret.push_back("");
            // get command, arguments, description
            QString com_str = get<COM_ALIAS>(current_command);
            QString arg_str = get<COM_ARGS>(current_command);
            int com_length = com_str.length();
            int arg_length = arg_str.length();
            QString des_str = get<COM_DESCRIPTION>(current_command);
            // deal with multi-commands and identically named commands
            QString com_str_original = com_str;
            QStringList new_multi_com_list = com_str.split(arg_separator);
            QString replace_str("");
            for(QString s : old_multi_com_list) {
                if(com_str.startsWith(s+arg_separator)) {
                    int rm_length = (s+arg_separator).length();
                    com_str.remove(0,rm_length);
                    replace_str += ".";
                    replace_str += QString(" ").repeated(rm_length-1);
                } else {
                    break;
                }
            }
            com_str.prepend(replace_str);
            old_multi_com_list = new_multi_com_list;
            // print identical command names (including aliases) with
            // different signatures only once
            if(com_str_original==old_com_str) {
                ret.back() += QString(".");
                ret.back() += QString(" ").repeated(max_com_length+space-1);
            } else {
                old_com_str = com_str_original;
                ret.back() += com_str;
                ret.back() += add_space(com_length,max_com_length+space);
            }
            ret.back() += arg_str;
            ret.back() += add_space(max_com_length+space+arg_length,max_com_length+space+max_arg_length+space);
            ret.back() += des_str;
        }
        return ret;
    }

    QString CommandCenter::get_help_string(int space) const {
        vector<QString> help_vector = get_help(space);
        QString ret;
        bool first = true;
        for(QString s : help_vector) {
            if(first) {
                first = false;
            } else {
                ret += "\n";
            }
            ret += s;
        }
        return ret;
    }

    QString CommandCenter::add_space(int from, int to) const {
        QString ret;
        for(int i = from; i<to; ++i) {
            switch(i%2) {
            case 0:
                ret += " ";
                break;
            case 1:
                ret += ".";
                break;
            case 2:
                ret += " ";
                break;
            default:
                ret += " ";
            }
        }
        return ret;
    }

    QString CommandCenter::execute(QString command_string) const {

        // remove leading and trailing whitespace
        command_string = command_string.simplified();

        // split multiple commands and process if necessary (also handles empty
        // command strings)
        QStringList command_string_list = command_string.split(command_separator);
        if(command_string_list.size()==0) {
            // empty command_string
            return "";
        } else if(command_string_list.size()>1) {
            QString ret("");
            bool first = true;
            for(QString com : command_string_list) {
                if(first) {
                    first = false;
                } else {
                    ret += "\n";
                }
                ret += this->execute(com);
            }
            return ret;
        } else {
            command_string = command_string_list[0];
        }

        // split command string into parts
        QStringList command_parts = command_string.split(arg_separator);
        if(command_parts.size()==0) {
            return "";
        }

        // search for matching command alias
        bool found = false;
        ReturnType res = {false,"Command not found"};

        // recombine to account for multipart commands
        QStringList command_name_list;
        QStringList command_arguments_list = command_parts;
        do {
            // take over one part from arguments to command name
            command_name_list.push_back(command_arguments_list.front());
            command_arguments_list.pop_front();

            // transform to string for comparison
            QString command_name = command_name_list.join(arg_separator);

            for(auto c : command_set) {
                if(get<COM_ALIAS>(c).contains(command_name)) {
                    found = true;
                    res = get<COM_FUNCTION>(c)->execute(command_arguments_list);
                    if(res.first) {
                        break;
                    }
                }
            }
        } while(command_arguments_list.size()>0);

        // report result
        if(!found) {
            return QString("Could not find command '%1'").arg(command_string);
        } else {
            return res.second;
        }
    }
}
