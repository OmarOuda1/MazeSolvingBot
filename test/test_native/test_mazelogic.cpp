#include <Arduino.h>
#include <unity.h>
#include "MazeLogic.h"

// Set up the test environment
void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_Is_Replay_Junction_Returns_True_For_Recorded_States(void) {
    // States 0, 1, 2, 4, 7 should return true
    TEST_ASSERT_TRUE(Is_Replay_Junction(0)); // 000
    TEST_ASSERT_TRUE(Is_Replay_Junction(1)); // 001
    TEST_ASSERT_TRUE(Is_Replay_Junction(2)); // 010
    TEST_ASSERT_TRUE(Is_Replay_Junction(4)); // 100
    TEST_ASSERT_TRUE(Is_Replay_Junction(7)); // 111
}

void test_Is_Replay_Junction_Returns_False_For_Non_Recorded_States(void) {
    // States 3, 5, 6 should return false
    TEST_ASSERT_FALSE(Is_Replay_Junction(3)); // 011
    TEST_ASSERT_FALSE(Is_Replay_Junction(5)); // 101
    TEST_ASSERT_FALSE(Is_Replay_Junction(6)); // 110
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_Is_Replay_Junction_Returns_True_For_Recorded_States);
    RUN_TEST(test_Is_Replay_Junction_Returns_False_For_Non_Recorded_States);
    UNITY_END();

    return 0;
}
