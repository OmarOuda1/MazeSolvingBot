#include <Arduino.h>

// ======= Sensors ======= //
void Maze_Solving_Task(void*);

TaskHandle_t maze_solving_task;

#define SENSORS_NUM 4 //Number of sensors used

// pin definitions
// Rename the sensors and change pin numbers
#define IR_1 1
#define IR_2 2
#define IR_3 3
#define IR_4 4



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
  
}

void loop() {

}

void Maze_Solving_Task(void* pvParameters) {
    // TODO
    // The values should be written as a distance value in cm
    // there are IR and ultrasonic sensors in use.
    // Read sensors values and implement the PID code to not hit the walls
    // then trigger maze solving logic on junctions and store these decitions in a string so it can be reduced later
    //

    while (true)
    {
        // NOTE: stack size = 8K byte

    }
    


}