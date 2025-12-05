#ifndef MAZE_LOGIC_H
#define MAZE_LOGIC_H

#include <Arduino.h>

// Enum for command types
enum CommandType {
    CMD_UNKNOWN,
    CMD_RC,
    CMD_START_SOLVING,
    CMD_LOAD_MAZE,
    CMD_ABORT,
    CMD_SETTINGS
};

// Struct to hold parsed command data
struct ParsedCommand {
    CommandType type;
    String data;
    int x; // For RC
    int y; // For RC
};

class MazeLogic {
public:
    static String Solve_Junction_Bits(uint8_t sensors_state);
    static ParsedCommand parseWebSocketMessage(String message);
};

#endif
