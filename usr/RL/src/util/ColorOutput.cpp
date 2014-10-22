#include "ColorOutput.h"

namespace ColorOutput {

    std::string set_attribute(ATTRIBUTE attr) {
        std::stringstream s;
        s << "\033[" << (int)attr << "m";
        return s.str();
    }

    std::string set_foreground(FOREGROUND fg) {
        std::stringstream s;
        s << "\033[" << (int)fg << "m";
        return s.str();
    }

    std::string set_background(BACKGROUND bg) {
        std::stringstream s;
        s << "\033[" << (int)bg << "m";
        return s.str();
    }

    std::string reset_all()        { return set_attribute(ATTRIBUTE::RESET_ALL); }

    std::string bold()             { return set_attribute(ATTRIBUTE::BOLD); }
    std::string dim()              { return set_attribute(ATTRIBUTE::DIM); }
    std::string underlined()       { return set_attribute(ATTRIBUTE::UNDERLINED); }
    std::string blink()            { return set_attribute(ATTRIBUTE::BLINK); }
    std::string reverse()          { return set_attribute(ATTRIBUTE::REVERSE); }
    std::string hidden()           { return set_attribute(ATTRIBUTE::HIDDEN); }
    std::string reset_bold()       { return set_attribute(ATTRIBUTE::RESET_BOLD); }
    std::string reset_dim()        { return set_attribute(ATTRIBUTE::RESET_DIM); }
    std::string reset_underlined() { return set_attribute(ATTRIBUTE::RESET_UNDERLINED); }
    std::string reset_blink()      { return set_attribute(ATTRIBUTE::RESET_BLINK); }
    std::string reset_reverse()    { return set_attribute(ATTRIBUTE::RESET_REVERSE); }
    std::string reset_hidden()     { return set_attribute(ATTRIBUTE::RESET_HIDDEN); }

    std::string fg_default()       { return set_foreground(FOREGROUND::DEFAULT); }
    std::string fg_black()         { return set_foreground(FOREGROUND::BLACK); }
    std::string fg_red()           { return set_foreground(FOREGROUND::RED); }
    std::string fg_green()         { return set_foreground(FOREGROUND::GREEN); }
    std::string fg_yellow()        { return set_foreground(FOREGROUND::YELLOW); }
    std::string fg_blue()          { return set_foreground(FOREGROUND::BLUE); }
    std::string fg_magenta()       { return set_foreground(FOREGROUND::MAGENTA); }
    std::string fg_cyan()          { return set_foreground(FOREGROUND::CYAN); }
    std::string fg_light_gray()    { return set_foreground(FOREGROUND::LIGHT_GRAY); }
    std::string fg_dark_gray()     { return set_foreground(FOREGROUND::DARK_GRAY); }
    std::string fg_light_red()     { return set_foreground(FOREGROUND::LIGHT_RED); }
    std::string fg_light_green()   { return set_foreground(FOREGROUND::LIGHT_GREEN); }
    std::string fg_light_yellow()  { return set_foreground(FOREGROUND::LIGHT_YELLOW); }
    std::string fg_light_blue()    { return set_foreground(FOREGROUND::LIGHT_BLUE); }
    std::string fg_light_magenta() { return set_foreground(FOREGROUND::LIGHT_MAGENTA); }
    std::string fg_light_cyan()    { return set_foreground(FOREGROUND::LIGHT_CYAN); }
    std::string fg_white()         { return set_foreground(FOREGROUND::WHITE); }

    std::string bg_default()       { return set_background(BACKGROUND::DEFAULT); }
    std::string bg_black()         { return set_background(BACKGROUND::BLACK); }
    std::string bg_red()           { return set_background(BACKGROUND::RED); }
    std::string bg_green()         { return set_background(BACKGROUND::GREEN); }
    std::string bg_yellow()        { return set_background(BACKGROUND::YELLOW); }
    std::string bg_blue()          { return set_background(BACKGROUND::BLUE); }
    std::string bg_magenta()       { return set_background(BACKGROUND::MAGENTA); }
    std::string bg_cyan()          { return set_background(BACKGROUND::CYAN); }
    std::string bg_light_gray()    { return set_background(BACKGROUND::LIGHT_GRAY); }
    std::string bg_dark_gray()     { return set_background(BACKGROUND::DARK_GRAY); }
    std::string bg_light_red()     { return set_background(BACKGROUND::LIGHT_RED); }
    std::string bg_light_green()   { return set_background(BACKGROUND::LIGHT_GREEN); }
    std::string bg_light_yellow()  { return set_background(BACKGROUND::LIGHT_YELLOW); }
    std::string bg_light_blue()    { return set_background(BACKGROUND::LIGHT_BLUE); }
    std::string bg_light_magenta() { return set_background(BACKGROUND::LIGHT_MAGENTA); }
    std::string bg_light_cyan()    { return set_background(BACKGROUND::LIGHT_CYAN); }
    std::string bg_white()         { return set_background(BACKGROUND::WHITE); }
};
