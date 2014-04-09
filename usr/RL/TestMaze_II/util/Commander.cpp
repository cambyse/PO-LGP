#include "Commander.h"

#include "debug.h"

using std::vector;
using std::get;
using std::max;
using std::shared_ptr;

namespace Commander {

    using namespace function_signature;

    ReturnType AbstractCommandFunction::wrong_number_of_parameters(int got, int need) const {
        return {false, QString("Wrong number of parameters (got %1, need %2)").arg(got).arg(need)};
    }

    QString CommandCenter::get_help(int space) const {
        // return
        QString ret("");
        // get maximum lengths
        int max_com_length = 0;
        int max_arg_length = 0;
        for(auto current_command : command_list) {
            max_com_length = max(max_com_length,get<0>(current_command).get_string().length());
            max_arg_length = max(max_arg_length,get<1>(current_command)->arg_description.length());
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
            ret += com_header;
            ret += add_space(running_pos+com_header.length(),running_pos+max_com_length+space);
            running_pos += max_com_length+space;
            ret += arg_header;
            ret += add_space(running_pos+arg_header.length(),running_pos+max_arg_length+space);
            running_pos += max_arg_length+space;
            ret += des_header;
        }
        // print commands
        for(auto current_command : command_list) {
            ret += "\n";
            int running_pos = 0;
            auto com_str = get<0>(current_command).get_string();
            auto arg_str = get<1>(current_command)->arg_description;
            int com_length = com_str.length();
            int arg_length = arg_str.length();
            auto des_str = get<2>(current_command).get_string();
            ret += com_str;
            ret += add_space(running_pos+com_length,running_pos+max_com_length+space);
            running_pos += max_com_length+space;
            ret += arg_str;
            ret += add_space(running_pos+arg_length,running_pos+max_arg_length+space);
            running_pos += max_arg_length+space;
            ret += des_str;
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

        // split command string
        QStringList command_parts = command_string.split(" ");
        if(command_parts.size()==0) {
            return "";
        }
        QString command_name = command_parts[0];
        command_parts.pop_front();

        // search for matching command alias
        bool found = false;
        ReturnType res = {false,"Command not found"};
        for(auto c : command_list) {
            if(get<0>(c)==command_name) {
                found = true;
                res = get<1>(c)->execute(command_parts);
                if(res.first) {
                    break;
                }
            }
        }

        // report result
        if(!found) {
            return QString("Could not find command '%1'").arg(command_name);
        } else {
            return res.second;
        }
    }
}
