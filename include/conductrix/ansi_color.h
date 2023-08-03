#ifndef ANSI_COLOR_H
#define ANSI_COLOR_H

// 文本颜色
#define ANSI_COLOR_BLACK "\033[30m"
#define ANSI_COLOR_RED "\033[31m"
#define ANSI_COLOR_GREEN "\033[32m"
#define ANSI_COLOR_YELLOW "\033[33m"
#define ANSI_COLOR_BLUE "\033[34m"
#define ANSI_COLOR_MAGENTA "\033[35m"
#define ANSI_COLOR_CYAN "\033[36m"
#define ANSI_COLOR_WHITE "\033[37m"

// 背景颜色
#define ANSI_BACKGROUND_BLACK "\033[40m"
#define ANSI_BACKGROUND_RED "\033[41m"
#define ANSI_BACKGROUND_GREEN "\033[42m"
#define ANSI_BACKGROUND_YELLOW "\033[43m"
#define ANSI_BACKGROUND_BLUE "\033[44m"
#define ANSI_BACKGROUND_MAGENTA "\033[45m"
#define ANSI_BACKGROUND_CYAN "\033[46m"
#define ANSI_BACKGROUND_WHITE "\033[47m"

// 样式
#define ANSI_STYLE_RESET "\033[0m"
#define ANSI_STYLE_BOLD "\033[1m"
#define ANSI_STYLE_UNDERLINE "\033[4m"
#define ANSI_STYLE_BLINK "\033[5m"

// 宏定义
#define SET_TEXT_COLOR(color) color
#define SET_BACKGROUND_COLOR(color) color
#define SET_STYLE(style) style

#define RESET_STYLE() ANSI_STYLE_RESET

#define COLORED_TEXT(text, color) SET_TEXT_COLOR(color) text RESET_STYLE()

#define MISSION_SWITCH_TO(text) COLORED_TEXT("Mission mode -> " text, ANSI_COLOR_YELLOW)

#define SUCCESS(text) COLORED_TEXT(text, "\033[1m" ANSI_COLOR_GREEN)

#endif // ANSI_COLOR_H
