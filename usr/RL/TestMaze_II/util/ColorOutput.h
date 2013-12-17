#include <iostream>

#ifndef COLOROUTPUT_H_
#define COLOROUTPUT_H_

class ColorOutput {
public:
    enum class ATTRIBUTE {
        BOLD          = 1,
        DIM           = 2,
        UNDERLINED    = 4,
        BLINK         = 5,
        REVERSE       = 7,
        HIDDEN        = 8
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
    ColorOutput() = default;
    virtual ~ColorOutput() = default;
    static void set_attribute(ATTRIBUTE attr) {
        std::cout << "\033[" << (int)attr << "m";
    }
    static void set_foreground(FOREGROUND fg) {
        std::cout << "\033[" << (int)fg << "m";
    }
    static void set_background(BACKGROUND bg) {
        std::cout << "\033[" << (int)bg << "m";
    }
    static void reset_all() {
        std::cout << "\033[0m";
    }
};

#endif /* COLOROUTPUT_H_ */
