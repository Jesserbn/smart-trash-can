#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

//declare task handlers
TaskHandle_t T1, T2, T3;
SemaphoreHandle_t xMutex;
QueueHandle_t xQopen, xQclose;

// Define tasks
void distance_calcul(void *pvParameters);
void open_can(void *pvParameters);
void close_can(void *pvParameters);

//hc 04 config 
const byte TRIGGER_PIN = 22; // Broche TRIGGER
const byte ECHO_PIN = 24;    // Broche ECHO
const byte Led = 26;

//instanciation d'un servo
Servo monServo;  

void setup() {
  Serial.begin(9600);
   //tasks , queues and semaphores 
   xTaskCreate(distance_calcul,"distance using hc sensor",128,NULL,1,&T1);
   xTaskCreate(open_can,"open ",128,NULL,1,&T2);
   xTaskCreate(close_can,"close",128,NULL,1,&T3);  
   xMutex = xSemaphoreCreateMutex();   
   xQopen = xQueueCreate(1, sizeof(long));
   xQclose = xQueueCreate(1, sizeof(long));

   //hc 04 initiation 
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW); 
    pinMode(ECHO_PIN, INPUT);

    //Led 
    pinMode (Led, OUTPUT);
    digitalWrite(Led,LOW);
   
   //servo motor config 
   monServo.attach(10);
}

void loop() {
}

void distance_calcul(void *pvParameters)
{  
  for(;;)
     {
       digitalWrite(TRIGGER_PIN, LOW);
       delayMicroseconds(2);
       digitalWrite(TRIGGER_PIN, HIGH);
       delayMicroseconds(10);
       digitalWrite(TRIGGER_PIN, LOW);
       // Read the echo time
        long duration = pulseIn(ECHO_PIN, HIGH);
         
       // Calculate the distance in cm
        long distance = duration * 0.034 / 2;
         long d = distance;
         Serial.println(distance);
        if ( distance <30 ) 
        {
          Serial.println("open");
          xQueueSendToBack(xQopen, &d, portMAX_DELAY);  
        }
        else 
        {
          Serial.println("close");
          xQueueSendToBack(xQclose, &d, portMAX_DELAY);  
        }
     
      vTaskDelay(1500/ portTICK_PERIOD_MS);
     }
}

void open_can(void *pvParameters)
{  
  long d ; 
  for(;;)
     {
     if( xQueueReceive(xQopen, &d, portMAX_DELAY)== pdTRUE)
     {
     if( xSemaphoreTake(xMutex, portMAX_DELAY)==pdTRUE)
     {
      digitalWrite(Led,HIGH);
      monServo.write(180);
      vTaskDelay(100/ portTICK_PERIOD_MS);
      xSemaphoreGive(xMutex);
     }
      vTaskDelay(1000/ portTICK_PERIOD_MS);      
     }
     }
}

void close_can(void *pvParameters)
{  
  long d ; 
  for(;;)
     {
      if( xQueueReceive(xQclose, &d, portMAX_DELAY)==pdTRUE)
      {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)==pdTRUE )
      {
      monServo.write(0);
      vTaskDelay(100/ portTICK_PERIOD_MS);
      digitalWrite(Led,LOW);
      xSemaphoreGive(xMutex);
      }
      vTaskDelay(1000/ portTICK_PERIOD_MS);
      
      }
     }
}
