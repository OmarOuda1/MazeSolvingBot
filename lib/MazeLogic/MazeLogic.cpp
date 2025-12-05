#include "MazeLogic.h"

String MazeLogic::Solve_Junction_Bits(uint8_t sensors_state) {
    // bit 2:Front, bit 1:Left, bit 0:Right
    switch (sensors_state)
    {
    case 0: // 000 no walls
        return "L"; // turn left to find a wall
    case 1: // 001 wall to the right
        return "L"; // turn left
    case 2: // 010 wall to the left
        return "S"; // go straight
    case 3: // 011 wall to the left and right
        return "S"; // go straight
    case 4: // 100 wall to the front
        return "L"; // turn left
    case 5: // 101 wall to the front and right
        return "L"; // turn left
    case 6: // 110 wall to the front and left
        return "R"; // turn right
    case 7: // 111 all walls
        return "B"; // go back
    default:
        return "S";
    }
}

ParsedCommand MazeLogic::parseWebSocketMessage(String message) {
    ParsedCommand cmd;
    cmd.type = CMD_UNKNOWN;
    cmd.x = 0;
    cmd.y = 0;

    int colonIndex = message.indexOf(':');
    String commandStr = (colonIndex != -1) ? message.substring(0, colonIndex) : message;
    String data = (colonIndex != -1) ? message.substring(colonIndex + 1) : "";

    cmd.data = data;

    if (commandStr == "rc") {
        cmd.type = CMD_RC;
        int commaIndex = data.indexOf(',');
        if (commaIndex != -1) {
            cmd.x = data.substring(0, commaIndex).toInt();
            cmd.y = data.substring(commaIndex + 1).toInt();
        }
    } else if (commandStr == "start_solving") {
        cmd.type = CMD_START_SOLVING;
    } else if (commandStr == "load_maze") {
        cmd.type = CMD_LOAD_MAZE;
    } else if (commandStr == "abort") {
        cmd.type = CMD_ABORT;
    } else if (commandStr == "settings") {
        cmd.type = CMD_SETTINGS;
    }

    return cmd;
}
