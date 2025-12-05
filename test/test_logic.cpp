#include <unity.h>
#include <MazeLogic.h>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_solve_junction_bits(void) {
    // Test all sensor states
    TEST_ASSERT_EQUAL_STRING("L", MazeLogic::Solve_Junction_Bits(0).c_str()); // 000
    TEST_ASSERT_EQUAL_STRING("L", MazeLogic::Solve_Junction_Bits(1).c_str()); // 001
    TEST_ASSERT_EQUAL_STRING("S", MazeLogic::Solve_Junction_Bits(2).c_str()); // 010
    TEST_ASSERT_EQUAL_STRING("S", MazeLogic::Solve_Junction_Bits(3).c_str()); // 011
    TEST_ASSERT_EQUAL_STRING("L", MazeLogic::Solve_Junction_Bits(4).c_str()); // 100
    TEST_ASSERT_EQUAL_STRING("L", MazeLogic::Solve_Junction_Bits(5).c_str()); // 101
    TEST_ASSERT_EQUAL_STRING("R", MazeLogic::Solve_Junction_Bits(6).c_str()); // 110
    TEST_ASSERT_EQUAL_STRING("B", MazeLogic::Solve_Junction_Bits(7).c_str()); // 111
}

void test_parse_rc(void) {
    ParsedCommand cmd = MazeLogic::parseWebSocketMessage("rc:100,200");
    TEST_ASSERT_EQUAL(CMD_RC, cmd.type);
    TEST_ASSERT_EQUAL(100, cmd.x);
    TEST_ASSERT_EQUAL(200, cmd.y);

    // Test negative values
    cmd = MazeLogic::parseWebSocketMessage("rc:-50,-10");
    TEST_ASSERT_EQUAL(CMD_RC, cmd.type);
    TEST_ASSERT_EQUAL(-50, cmd.x);
    TEST_ASSERT_EQUAL(-10, cmd.y);
}

void test_parse_start_solving(void) {
    ParsedCommand cmd = MazeLogic::parseWebSocketMessage("start_solving:Maze1");
    TEST_ASSERT_EQUAL(CMD_START_SOLVING, cmd.type);
    TEST_ASSERT_EQUAL_STRING("Maze1", cmd.data.c_str());
}

void test_parse_load_maze(void) {
    ParsedCommand cmd = MazeLogic::parseWebSocketMessage("load_maze:RSLB");
    TEST_ASSERT_EQUAL(CMD_LOAD_MAZE, cmd.type);
    TEST_ASSERT_EQUAL_STRING("RSLB", cmd.data.c_str());
}

void test_parse_abort(void) {
    ParsedCommand cmd = MazeLogic::parseWebSocketMessage("abort");
    TEST_ASSERT_EQUAL(CMD_ABORT, cmd.type);
}

void test_parse_settings(void) {
    String json = "{\"max_speed\":100,\"kp\":2.0}";
    String msg = "settings:" + json;
    ParsedCommand cmd = MazeLogic::parseWebSocketMessage(msg);
    TEST_ASSERT_EQUAL(CMD_SETTINGS, cmd.type);
    TEST_ASSERT_EQUAL_STRING(json.c_str(), cmd.data.c_str());
}

void test_parse_unknown(void) {
    ParsedCommand cmd = MazeLogic::parseWebSocketMessage("foobar:baz");
    TEST_ASSERT_EQUAL(CMD_UNKNOWN, cmd.type);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_solve_junction_bits);
    RUN_TEST(test_parse_rc);
    RUN_TEST(test_parse_start_solving);
    RUN_TEST(test_parse_load_maze);
    RUN_TEST(test_parse_abort);
    RUN_TEST(test_parse_settings);
    RUN_TEST(test_parse_unknown);
    UNITY_END();

    return 0;
}
