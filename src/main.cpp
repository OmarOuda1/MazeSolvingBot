#include <Arduino.h>

// ======= Sensors ======= //
void Sensors_Task(void*);

TaskHandle_t sensors_task;
QueueHandle_t sensors_queue;

#define QUEUE_LENGTH 5

#define IR_NUM 4 //Number of IR sensors use


void setup() {
    // ======= Motors ======= //
    sensors_queue = xQueueCreate( QUEUE_LENGTH , sizeof(float)*IR_NUM );
    xTaskCreatePinnedToCore(Sensors_Task,      // Function name
                 "Sensors_Task",   // Task name
                 1024,            // Stack size in words (Word = 4 bytes)
                 NULL,            // Task parameters
                 2,               // Task priority (From 0 to 24)
                 &sensors_task,     // Pointer to task handle
                 APP_CPU_NUM
                );
  
}

void loop() {

}

void Sensors_Task(void* pvParameters) {
    // TODO
    // Create the code that takes values from a queue and execute those values on the motors
    // The number of motors is 4 but each two is wired together so the number of motors you can control is 2
    // The values are in the from of x and y values use these values to determine the speed of the two motors
    // Use a library to manage the motors dsend any signals yourself just use the library
    // You objective is to use the x and y values to change the speed of the motors.

    // Examples
    // x = 100 means move forward at full speed -100 means backwards
    // if y = 100 and x = 0 that means rotate around it axis clockwise ie. motors run in opposite directions
    // y = -100 counter clockwise

    while (true)
    {
        float ir_values[IR_NUM];
    
        // TODO Take values from the sensors and store them in the values array (NOTE: This while loop is equivalent to void loop)
        // NOTE: stack size = 4K byte

        xQueueSend(sensors_queue, ir_values, portMAX_DELAY);
    }
    


}