#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

const char *ssid = "ESP32-Robot-Controller";
const char *password = "1234567-8";
bool isSolving; 
int maxSpeed = 50;

// ======= MazeSolving ======= //
#include <NewPing.h>
void Maze_Solving_Task(void*);

TaskHandle_t maze_solving_task;

#define SENSORS_NUM 4 //Number of sensors used
#define MAX_DISTANCE_IR 100
#define MIN_DISTANCE 2

// pin definitions
// Rename the sensors and change pin numbers
#define RIGHT_IR 1
#define LEFT_IR 2
#define TRIG 5
#define ECHO 18
#define MAX_DISTANCE 200

NewPing front_ultra(TRIG, ECHO, MAX_DISTANCE);

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


// Pin definition
#define EN_A 32
#define IN1_A 14
#define IN2_A 27

#define IN3_B 26
#define IN4_B 25
#define EN_B 33

#define PWM_MOTOR_FREQUENCY   100 // Increase PWM frequency in the future.
#define PWM_MOTOR_RESOLUTION    8

L293D rightmotor(IN1_A,IN2_A,EN_A,0);
L293D leftmotor(IN3_B,IN4_B,EN_B,1);


// ======= WebSocket ======= //

WebSocketsServer webSocket = WebSocketsServer(81);
void handleSettings(String payload); 
void handleRC(String payload); 
void handleStartSolving(String mazeName); 
void sendSolution(); 
void handleLoadMaze(String mazeSolution); 
void handleAbort(); 
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length); 

void setup() {
    // ======= WebSocket ======= //

    // ======= Sensors ======= //
    // sensors_queue = xQueueCreate( QUEUE_LENGTH , sizeof(float)*SENSORS_NUM );
    xTaskCreatePinnedToCore(Maze_Solving_Task,      // Function name
                 "Maze_Solving_Task",   // Task name
                 4096,            // Stack size in words (Word = 4 bytes)
                 NULL,            // Task parameters
                 2,               // Task priority (From 0 to 24)
                 &maze_solving_task,     // Pointer to task handle
                 APP_CPU_NUM
                );
    vTaskSuspend(maze_solving_task);

    pinMode(RIGHT_IR,INPUT);
    pinMode(LEFT_IR,INPUT);

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

  
}

void loop() {

}

String Solve_Junction_Bits(uint8_t sensors_state) {
    // bit 2:Front, bit 1:Left, bit 0:Right
    switch (sensors_state)
    {
    case 0: // 000 no walls
        return "R"; // turn right to find a wall
    case 1: // 001 wall to the right
        return "S"; // go straight
    case 2: // 010 wall to the left
        return "R"; // turn right
    case 3: // 011 wall to the left and right
        return "S"; // go straight
    case 4: // 100 wall to the front
        return "R"; // turn right
    case 5: // 101 wall to the front and right
        return "L"; // go left
    case 6: // 110 wall to the front and left
        return "R"; // turn right
    case 7: // 111 all walls
        return "B"; // go back
    default:
        return "S";
    }
}


void Move_Radius(char Direction,int val,point* cmd) {
    
    switch (Direction)
    {
    case 'R':
        cmd->y  = val;
        break;
    case 'L':
        cmd->y  = -val;
        break;
    }
    xQueueSend(motors_queue, &cmd, portMAX_DELAY);
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
        int left_ir = analogRead(LEFT_IR);
        int right_ir = analogRead(RIGHT_IR);
        float front_us = front_ultra.ping_cm();

        uint8_t sensors_state = 0;
        sensors_state |= (right_ir < MAX_DISTANCE_IR) ? (1 << 0) : 0;
        sensors_state |= (left_ir < MAX_DISTANCE_IR) ? (1 << 1) : 0;
        sensors_state |= (front_us < MIN_DISTANCE) ? (1 << 2) : 0;

        if ((sensors_state ^ 6) | (sensors_state ^ 5) | sensors_state | (sensors_state ^ 3) ) {
            String next;
            next = Solve_Junction_Bits(sensors_state);
            path += next;
            if (next == "S") {
                //TODO add motor base speed here
                //TODO send motors value to the Queue
            } else if (next == "B") {
                //TODO rotate 180°
            } else {
                char direction = next.charAt(0);
                Move_Radius(direction, 5, &cmd);
            }
        } else if (sensors_state == 2){
            //PID
            Input = left_ir - right_ir;
            Wall_PID.Compute();
            cmd.x = maxSpeed;
            cmd.y = Output;
            xQueueSend(motors_queue, &cmd, portMAX_DELAY);
        } else {
            // solve junction without storing the result
        }
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

        leftmotor.SetMotorSpeed(leftSpeed);
        rightmotor.SetMotorSpeed(rightSpeed);
    }
    


}

void handleSettings(String payload) {
    StaticJsonDocument<200> doc;
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
    int commaIndex = payload.indexOf(',');
    if (commaIndex != -1) {
        int x = payload.substring(0, commaIndex).toInt();
        int y = payload.substring(commaIndex + 1).toInt();
        Serial.printf("RC command: x=%d, y=%d\n", x, y);
        // Add motor control logic here
    }
}

void handleStartSolving(String mazeName) {
    Serial.printf("Start solving command received for maze: %s\n", mazeName.c_str());
    isSolving = true;
    vTaskResume(maze_solving_task);
}
void sendSolution() {
    // if (!isSolving) {
    //     String message = "maze_solution:" + mazeName + ";" + path;
    //     webSocket.broadcastTXT(message);        
    // }
}

void handleLoadMaze(String mazeSolution) {
    Serial.printf("Load maze command received with solution: %s\n", mazeSolution.c_str());
    // Add logic to execute the maze solution
}

void handleAbort() {
    Serial.println("Abort command received");
    isSolving = false;
    // Add any other necessary cleanup or motor stop logic here
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
            Serial.printf("[%u] get Text: %s\n", num, payload);
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