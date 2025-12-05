#ifndef MAZELOGIC_H
#define MAZELOGIC_H

#include <Arduino.h>

// Function to determine the next move based on sensor state during exploration
String Solve_Junction_Bits(uint8_t sensors_state, String &path);

// Function to determine if the robot is at a decision point (junction) during replay
bool Is_Replay_Junction(uint8_t sensors_state);

#endif
