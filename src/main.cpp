#include <Arduino.h>

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



void setup() {
    // ======= Sensors ======= //
    // sensors_queue = xQueueCreate( QUEUE_LENGTH , sizeof(float)*SENSORS_NUM );
    xTaskCreatePinnedToCore(Maze_Solving_Task,      // Function name
                 "Maze_Solving_Task",   // Task name
                 2048,            // Stack size in words (Word = 4 bytes)
                 NULL,            // Task parameters
                 2,               // Task priority (From 0 to 24)
                 &maze_solving_task,     // Pointer to task handle
                 APP_CPU_NUM
                );

    pinMode(RIGHT_IR,INPUT);
    pinMode(LEFT_IR,INPUT);

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

String Solve_Junction(float right,float left,float front) {
    if (right > MAX_DISTANCE_IR) {
        return "R";
    }
    else if (front > MIN_DISTANCE) {
        return "S";
    }
    else if (left > MAX_DISTANCE_IR) {
        return "L";
    }
    else {
        return "B";
    }
}

void Move_Radius(char Direction,int val) {
    point cmd;
    
    xQueueSend(motors_queue, &cmd, portMAX_DELAY);
}

void Maze_Solving_Task(void* pvParameters) {
    // TODO
    // The values should be written as a distance value in cm
    // there are IR and ultrasonic sensors in use.
    // Read sensors values and implement the PID code to not hit the walls
    // then trigger maze solving logic on junctions and store these decitions in a string so it can be reduced later
    // LSRB

    while (true)
    {
        // NOTE: stack size = 8K byte

        // read sensors 
        float left_val = analogRead(LEFT_IR);
        float right_val = analogRead(LEFT_IR);
        float front_val = front_ultra.ping_cm();
        if (right_val > MAX_DISTANCE_IR || left_val > MAX_DISTANCE_IR || front_val < MIN_DISTANCE) {
            
            path += Solve_Junction(right_val,left_val,front_val); 
            Move_Radius('L',5);
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
        int leftSpeed = cmd.x - cmd.y;
        int rightSpeed = cmd.x + cmd.y;

        leftmotor.SetMotorSpeed(leftSpeed);
        rightmotor.SetMotorSpeed(rightSpeed);
    }
    


}