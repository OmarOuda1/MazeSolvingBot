#include "MazeLogic.h"

String Solve_Junction_Bits(uint8_t sensors_state, String &path) {
    // bit 2:Front, bit 1:Left, bit 0:Right
    switch (sensors_state)
    {
    case 0: // 000 no walls
        path += "L";
        return "L"; // turn left to find a wall
    case 1: // 001 wall to the right
        path += "L";
        return "L"; // turn left
    case 2: // 010 wall to the left
        path += "S";
        return "S"; // go straight
    case 3: // 011 wall to the left and right
        return "S"; // go straight
    case 4: // 100 wall to the front
        path += "L";
        return "L"; // turn left
    case 5: // 101 wall to the front and right
        return "L"; // turn left
    case 6: // 110 wall to the front and left
        return "R"; // turn right
    case 7: // 111 all walls
        path += "B";
        return "B"; // go back
    default:
        return "S";
    }
}

bool Is_Replay_Junction(uint8_t sensors_state) {
    // A junction in replay mode is any state where we recorded a decision.
    // Solve_Junction_Bits records a move (path += ...) for states 0, 1, 2, 4, 7.
    // It does not record for 3, 5, 6.
    return (sensors_state != 3 && sensors_state != 5 && sensors_state != 6);
}
