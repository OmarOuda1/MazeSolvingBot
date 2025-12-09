#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <SPIFFS.h>

const char *ssid = "ESP32-Robot-Controller";
const char *password = "1234567-8";
bool isSolving = false;
int maxSpeed = 50;
String currentMazeName = "";
String replayPath = "";
bool isReplaying = false;
int replayIndex = 0;

// ======= MazeSolving ======= //
#include <NewPing.h>
void Maze_Solving_Task(void*);

TaskHandle_t maze_solving_task;

#define SENSORS_NUM 4 //Number of sensors used
#define MAX_DISTANCE_IR 100
#define MIN_DISTANCE 2

// pin definitions
#define IR_1 36 // Left IR
#define IR_2 39 // Right IR
#define IR_3 34 // Not used yet
#define IR_4 35 // Not used yet

#define ECHO_1 25
#define TRIG_1 26
#define ECHO_2 32 // Not used yet
#define TRIG_2 33 // Not used yet

#define IN1 19
#define IN2 18
#define IN3 17
#define IN4 16
#define ENA 23
#define ENB 4

#define MAX_DISTANCE 200

NewPing front_ultra(TRIG_1, ECHO_1, MAX_DISTANCE);

String path = "";

// ======= PID ======= //
#include <PID_v1.h>
double Setpoint = 0, Input, Output;

double Kp=2, Ki=5, Kd=1;
PID Wall_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// ======= Motors ======= //
#include <L293D.h>
void Motors_Task(void*);

TaskHandle_t motors_task;
QueueHandle_t motors_queue;

#define QUEUE_SIZE 5

struct point
{
    int8_t x = 0;
    int8_t y = 0;
};


#define PWM_MOTOR_FREQUENCY   100 // Increase PWM frequency in the future.
#define PWM_MOTOR_RESOLUTION    8

L293D rightmotor(IN1,IN2,ENA,0);
L293D leftmotor(IN3,IN4,ENB,1);


// ======= Web Server & WebSocket ======= //
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

void handleSettings(String payload); 
void handleRC(String payload); 
void handleStartSolving(String mazeName); 
void sendSolution(); 
void optimizePath(String &path);
void handleLoadMaze(String mazeSolution); 
void handleAbort(); 
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length); 

// ======= Function Prototypes ======= //
String Solve_Junction_Bits(uint8_t sensors_state);
void Perform_Turn(char direction, int duration, point* cmd);

void setup() {
    Serial.begin(115200);
    delay(100);

    // ======= Sensors ======= //
    pinMode(IR_2,INPUT);
    pinMode(IR_1,INPUT);

    Wall_PID.SetOutputLimits(-maxSpeed, maxSpeed);
    Wall_PID.SetMode(AUTOMATIC);

    // ======= Motors ======= //
    motors_queue = xQueueCreate( QUEUE_SIZE , sizeof(point) );
    xTaskCreatePinnedToCore(Motors_Task,     // Function name
                            "Motors_Task",   // Task name
                            1024,            // Stack size in words (Word = 4 bytes)
                            NULL,            // Task parameters
                            2,               // Task priority (From 0 to 24)
                            &motors_task ,   // Pointer to task handle
                            APP_CPU_NUM      // Core number
                            );
    
    leftmotor.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
    rightmotor.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);

    leftmotor.SetMotorSpeed(0);
    rightmotor.SetMotorSpeed(0);

    // ======= WiFi & Server ======= //
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    server.serveStatic("/", SPIFFS, "/index.html");
    server.serveStatic("/style.css", SPIFFS, "/style.css");
    server.serveStatic("/script.js", SPIFFS, "/script.js");
    server.serveStatic("/vendor", SPIFFS, "/vendor");
    server.serveStatic("/fonts", SPIFFS, "/fonts");

    server.begin();
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);

    // ======= Tasks ======= //
    xTaskCreatePinnedToCore(Maze_Solving_Task,      // Function name
                 "Maze_Solving_Task",   // Task name
                 4096,            // Stack size in words (Word = 4 bytes)
                 NULL,            // Task parameters
                 2,               // Task priority (From 0 to 24)
                 &maze_solving_task,     // Pointer to task handle
                 APP_CPU_NUM
                );
    vTaskSuspend(maze_solving_task);
    handleAbort();
}

void loop() {
    webSocket.loop();
    server.handleClient();
}

String Solve_Junction_Bits(uint8_t sensors_state) {
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


void Perform_Turn(char direction, int duration, point* cmd) {
    // 1. Align: Move forward slightly to center wheels in junction
    cmd->x = 40; // Moderate speed
    cmd->y = 0;
    xQueueSend(motors_queue, cmd, portMAX_DELAY);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 2. Turn: Spot turn (rotate in place)
    cmd->x = 0;
    if (direction == 'R') {
        cmd->y = maxSpeed;
    } else if (direction == 'L') {
        cmd->y = -maxSpeed; // Left Turn
    }
    xQueueSend(motors_queue, cmd, portMAX_DELAY);
    vTaskDelay(duration / portTICK_PERIOD_MS);

    // 3. Stop: Stabilize before resuming PID
    cmd->x = 0;
    cmd->y = 0;
    xQueueSend(motors_queue, cmd, portMAX_DELAY);
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

void Maze_Solving_Task(void* pvParameters) {
    // TODO
    // The values should be written as a distance value in cm
    // there are IR and ultrasonic sensors in use.
    // Read sensors values and implement the PID code to not hit the walls
    // then trigger maze solving logic on junctions and store these decitions in a string so it can be reduced later
    // LSRB

    point cmd;    
    while (true)
    {
        // NOTE: stack size = 8K byte

        // read sensors 
        int left_ir = analogRead(IR_1);
        int right_ir = analogRead(IR_2);
        float front_us = front_ultra.ping_cm();

        uint8_t sensors_state = 0;
        sensors_state |= (right_ir < MAX_DISTANCE_IR) ? (1 << 0) : 0;
        sensors_state |= (left_ir < MAX_DISTANCE_IR) ? (1 << 1) : 0;
        sensors_state |= (front_us < MIN_DISTANCE) ? (1 << 2) : 0;

        String next = "";
        bool atJunction = false;

        // Check if we are at a junction/special state (anything other than simple walls)
        // Simplistic check: If sensors are not "Left and Right" (3) or "Left only" (2)
        // Adjust this logic based on actual wall configurations.
        // Original logic was: ((sensors_state ^ 6) | (sensors_state ^ 5) | sensors_state | (sensors_state ^ 3))
        // Which is weird. Let's assume junction is when front wall exists OR no walls OR ...
        // For Replay, we need to detect the decision point.
        // Let's rely on the exploration logic's junction detection for consistency.

        if (isReplaying) {
             // In Replay Mode, we follow the path string
             // We need to detect when we are at a junction.
            if ((sensors_state ^ 6) | (sensors_state ^ 5) | sensors_state | (sensors_state ^ 3)) {
                // We are at a decision point (or end of corridor)
                if (replayIndex < replayPath.length()) {
                    char move = replayPath.charAt(replayIndex++);
                    next = String(move);
                } else {
                    // Path finished
                    handleAbort();
                    continue;
                }
            } else {
                next = "S";
            }
        } else {
            // Exploration Mode
            next = Solve_Junction_Bits(sensors_state);
        }

        if (next == "S") {
            // PID Control
            Input = left_ir - right_ir;
            Setpoint = 0;

            Wall_PID.Compute();
            cmd.x = maxSpeed;
            cmd.y = Output;
            xQueueSend(motors_queue, &cmd, portMAX_DELAY);
        } else if (next == "B") {
            // Rotate 180 (Turn Right)
            Perform_Turn('R', 600, &cmd);
        } else {
            char direction = next.charAt(0);
            Perform_Turn(direction, 300, &cmd);
        }

        // Check for end of maze condition (e.g., specific sensor pattern or manual stop)
        // If exploring and done, send solution
        // For now, assume user stops it or we add a specific condition later.

        // Small delay to prevent CPU hogging and queue flooding
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void Motors_Task(void* pvParameters) {
    // TODO
    // Create the code that takes values from a queue and execute those values on the motors
    // The number of motors is 4 but each two is wired together so the number of motors you can control is 2
    // The values are in the from of x and y values use these values to determine the speed of the two motors
    // Use a library to manage the motors do not send any signals yourself just use the library
    // You objective is to use the x and y values to change the speed of the motors.

    // Examples
    // x = 100 means move forward at full speed -100 means backwards
    // if y = 100 and x = 0 that means rotate around it axis clockwise ie. motors run in opposite directions
    // y = -100 counter clockwise

    while (true)
    {
        point cmd;
    
        xQueueReceive(motors_queue, &cmd, portMAX_DELAY);

        // TODO move motors (NOTE: This while loop is equivalent to void loop)
        // NOTE: stack size = 4K byte
        int leftSpeed = cmd.x + cmd.y;
        int rightSpeed = cmd.x - cmd.y;
        Serial.println("Moved");

        leftmotor.SetMotorSpeed(leftSpeed);
        rightmotor.SetMotorSpeed(rightSpeed);
    }
    


}

void handleSettings(String payload) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }

    maxSpeed = doc["max_speed"];
    Kp = doc["kp"];
    Ki = doc["ki"];
    Kd = doc["kd"];

    Wall_PID.SetTunings(Kp, Ki, Kd);
    Wall_PID.SetOutputLimits(-maxSpeed, maxSpeed);

    Serial.printf("Settings updated: max_speed=%d, Kp=%.2f, Ki=%.2f, Kd=%.2f\n", maxSpeed, Kp, Ki, Kd);
}

void handleRC(String payload) {
    // If we are solving, stop it first to take manual control
    if (isSolving) {
        vTaskSuspend(maze_solving_task);
        isSolving = false;
        Serial.println("Paused solving for RC");
    }

    int commaIndex = payload.indexOf(',');
    if (commaIndex != -1) {
        int x = payload.substring(0, commaIndex).toInt();
        int y = payload.substring(commaIndex + 1).toInt();
        // Serial.printf("RC command: x=%d, y=%d\n", x, y);

        point cmd;
        cmd.x = x;
        cmd.y = y;
        Serial.println("Sent x and y to motors");
        xQueueSend(motors_queue, &cmd, portMAX_DELAY);
    }
}

void handleStartSolving(String mazeName) {
    Serial.printf("Start solving command received for maze: %s\n", mazeName.c_str());
    currentMazeName = mazeName;
    path = "";
    isSolving = true;
    isReplaying = false;
    vTaskResume(maze_solving_task);
}

void optimizePath(String &path) {
    bool changed = true;
    while (changed) {
        changed = false;
        if (path.indexOf("LBR") != -1) {
            path.replace("LBR", "B");
            changed = true;
        }
        if (path.indexOf("LBS") != -1) {
            path.replace("LBS", "R");
            changed = true;
        }
        if (path.indexOf("RBL") != -1) {
            path.replace("RBL", "B");
            changed = true;
        }
        if (path.indexOf("SBL") != -1) {
            path.replace("SBL", "R");
            changed = true;
        }
        if (path.indexOf("SBS") != -1) {
            path.replace("SBS", "B");
            changed = true;
        }
        if (path.indexOf("LBL") != -1) {
            path.replace("LBL", "S");
            changed = true;
        }
    }
}

void sendSolution() {
     if (isSolving && !isReplaying && path.length() > 0) {
         optimizePath(path);
         String message = "maze_solution:" + currentMazeName + ";" + path;
         webSocket.broadcastTXT(message);
         Serial.println("Sent solution: " + message);
     }
}

void handleLoadMaze(String mazeSolution) {
    Serial.printf("Load maze command received with solution: %s\n", mazeSolution.c_str());
    replayPath = mazeSolution;
    replayIndex = 0;
    isReplaying = true;
    isSolving = true;
    vTaskResume(maze_solving_task);
}

void handleAbort() {
    Serial.println("Abort command received");
    isSolving = false;
    isReplaying = false;
    vTaskSuspend(maze_solving_task);

    // Stop motors
    point cmd;
    xQueueSend(motors_queue, &cmd, portMAX_DELAY);

    // If we aborted during exploration, we might want to send what we have,
    // but usually abort means "stop everything".
    // Check if we found a solution before aborting?
    // sendSolution(); // Optional
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            break;
        }
        case WStype_TEXT: {
            // Serial.printf("[%u] get Text: %s\n", num, payload);
            String message = String((char *)payload);
            int colonIndex = message.indexOf(':');
            String command = (colonIndex != -1) ? message.substring(0, colonIndex) : message;
            String data = (colonIndex != -1) ? message.substring(colonIndex + 1) : "";

            if (command == "rc") {
                handleRC(data);
            } else if (command == "start_solving") {
                handleStartSolving(data);
            } else if (command == "load_maze") {
                handleLoadMaze(data);
            } else if (command == "abort") {
                handleAbort();
            } else if (command == "settings") {
                handleSettings(data);
            } else {
                Serial.println("Unknown command received");
            }
            break;
        }
        case WStype_BIN:
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}
