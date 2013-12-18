#ifndef COLOROUTPUT_H_
#define COLOROUTPUT_H_

#include <sstream>

namespace ColorOutput {

    enum class ATTRIBUTE {
        RESET_ALL        = 0,
        BOLD             = 1,
        DIM              = 2,
        UNDERLINED       = 4,
        BLINK            = 5,
        REVERSE          = 7,
        HIDDEN           = 8,
        RESET_BOLD       = 21,
        RESET_DIM        = 22,
        RESET_UNDERLINED = 24,
        RESET_BLINK      = 25,
        RESET_REVERSE    = 27,
        RESET_HIDDEN     = 28
        };

    enum class FOREGROUND {
        DEFAULT       = 39,
        BLACK         = 30,
        RED           = 31,
        GREEN         = 32,
        YELLOW        = 33,
        BLUE          = 34,
        MAGENTA       = 35,
        CYAN          = 36,
        LIGHT_GRAY    = 37,
        DARK_GRAY     = 90,
        LIGHT_RED     = 91,
        LIGHT_GREEN   = 92,
        LIGHT_YELLOW  = 93,
        LIGHT_BLUE    = 94,
        LIGHT_MAGENTA = 95,
        LIGHT_CYAN    = 96,
        WHITE         = 97
        };

    enum class BACKGROUND {
        DEFAULT       = 49,
        BLACK         = 40,
        RED           = 41,
        GREEN         = 42,
        YELLOW        = 43,
        BLUE          = 44,
        MAGENTA       = 45,
        CYAN          = 46,
        LIGHT_GRAY    = 47,
        DARK_GRAY     = 100,
        LIGHT_RED     = 101,
        LIGHT_GREEN   = 102,
        LIGHT_YELLOW  = 103,
        LIGHT_BLUE    = 104,
        LIGHT_MAGENTA = 105,
        LIGHT_CYAN    = 106,
        WHITE         = 107
        };

    std::string set_attribute(ATTRIBUTE attr);

    std::string set_foreground(FOREGROUND fg);

    std::string set_background(BACKGROUND bg);

    std::string reset_all();

    std::string bold();
    std::string dim();
    std::string underlined();
    std::string blink();
    std::string reverse();
    std::string hidden();
    std::string reset_bold();
    std::string reset_dim();
    std::string reset_underlined();
    std::string reset_blink();
    std::string reset_reverse();
    std::string reset_hidden();

    std::string fg_default();
    std::string fg_black();
    std::string fg_red();
    std::string fg_green();
    std::string fg_yellow();
    std::string fg_blue();
    std::string fg_magenta();
    std::string fg_cyan();
    std::string fg_light_gray();
    std::string fg_dark_gray();
    std::string fg_light_red();
    std::string fg_light_green();
    std::string fg_light_yellow();
    std::string fg_light_blue();
    std::string fg_light_magenta();
    std::string fg_light_cyan();
    std::string fg_white();

    std::string bg_default();
    std::string bg_black();
    std::string bg_red();
    std::string bg_green();
    std::string bg_yellow();
    std::string bg_blue();
    std::string bg_magenta();
    std::string bg_cyan();
    std::string bg_light_gray();
    std::string bg_dark_gray();
    std::string bg_light_red();
    std::string bg_light_green();
    std::string bg_light_yellow();
    std::string bg_light_blue();
    std::string bg_light_magenta();
    std::string bg_light_cyan();
    std::string bg_white();
};

#endif /* COLOROUTPUT_H_ */
