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

// vector<QString> CommandCenter::print_commands(int space) const {
//     // return
//     vector<QString> ret;
//     // get maximum lengths
//     int max_com_length = 0;
//     int max_man_length = 0;
//     int max_opt_length = 0;
//     for(auto current_command : commands) {
//         max_com_length = max(max_com_length,get<0>(current_command).get_string().length());
//         max_man_length = max(max_man_length,get<1>(current_command)->get_string().length()+2);
//         max_opt_length = max(max_opt_length,get<2>(current_command)->get_string().length());
//     }
//     // adjust for header
//     QString com_header("COMMAND");
//     QString man_header("MANDATORY ARGS");
//     QString opt_header("OPTIONAL ARGS");
//     QString des_header("DESCRIPTION");
//     max_com_length = max(max_com_length,com_header.length());
//     max_man_length = max(max_man_length,man_header.length());
//     max_opt_length = max(max_opt_length,opt_header.length());
//     // print header
//     ret.push_back("");
//     ret.back() +=
//         com_header + add_space(com_header.length(),max_com_length+space) +
//         man_header + add_space(man_header.length(),max_man_length+space) +
//         opt_header + add_space(opt_header.length(),max_opt_length+space) +
//         des_header;
//     // print commands
//     for(auto current_command : commands) {
//         ret.push_back("");
//         QString& out = ret.back();
//         int running_pos = 0;
//         auto com_str = get<0>(current_command).get_string();
//         auto man_str = get<1>(current_command)->get_string();
//         auto opt_str = get<2>(current_command)->get_string();
//         opt_str = opt_str.length()==0?opt_str:("["+opt_str+"]");
//         int com_length = com_str.length();
//         int man_length = man_str.length();
//         int opt_length = opt_str.length();
//         auto des_str = get<3>(current_command).get_string();
//         out += com_str + add_space(com_length+running_pos,max_com_length+space+running_pos);
//         running_pos += max_com_length+space;
//         out += man_str + add_space(man_length+running_pos,max_man_length+space+running_pos);
//         running_pos += max_man_length+space;
//         out += opt_str + add_space(opt_length+running_pos,max_opt_length+space+running_pos);
//         running_pos += max_opt_length+space;
//         out += des_str;
//     }
//     return ret;
// }

// QString CommandCenter::add_space(int from, int to) const {
//     QString ret;
//     for(int i = from; i<to; ++i) {
//         if(i%2==0) {
//             ret += ".";
//         } else {
//             ret += " ";
//         }
//     }
//     return ret;
// }

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
