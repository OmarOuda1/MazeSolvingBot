#include <Arduino.h>

// ======= Sensors ======= //
void Sensors_Task(void*);

TaskHandle_t sensors_task;
QueueHandle_t sensors_queue;

#define QUEUE_LENGTH 5

#define SENSORS_NUM 4 //Number of sensors used

// pin definitions
#define IR_1 1
#define IR_2 2
#define IR_3 3
#define IR_4 4



void setup() {
    // ======= Sensors ======= //
    sensors_queue = xQueueCreate( QUEUE_LENGTH , sizeof(float)*SENSORS_NUM );
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
    // Create the code that wrtites values to a queue
    // The values should be written as a distance value in cm
    // there are IR and ultrasonic sensors in use.
    // use #define to give every member of the array a name to make it easy

    while (true)
    {
        float sensors_values[SENSORS_NUM];
    
        // TODO Take values from the sensors and store them in the values array (NOTE: This while loop is equivalent to void loop)
        // NOTE: stack size = 4K byte

        xQueueSend(sensors_queue, sensors_values, portMAX_DELAY);
    }
    


}