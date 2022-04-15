#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define RED 12
#define GREEN 25
#define BUTTON_1 34 //
#define SQUARE_INPUT 27 //SQUARE WAVE
#define DC_INPUT 35  //POTENTIOMETER


// TIME INTERVALS

int task1_period = 17;
int task2_period = 200;
int task3_period = 1000;
int task4_period = 42;
int task5_period = 42;
int task6_period = 100;
int task7_period = 333;
int task8_period = 333;
int task9_period = 5000;

// prototype of tasks functions
void task1( void *pvParameters );
void task2( void *pvParameters );
void task3( void *pvParameters );
void task4( void *pvParameters );
void task5( void *pvParameters );
void task6( void *pvParameters );
void task7( void *pvParameters );
void task8( void *pvParameters );
void task9( void *pvParameters );

// STRUCTURE TYPE
struct dataTasks{
  int buttonState;
  int freqResult;
  int analogValue;
  int analogAvg;
  int errorCode;
 
};

QueueHandle_t errorQueue = xQueueCreate(1,sizeof(int));
SemaphoreHandle_t mutex;
int analogs[4] = {0,0,0,0};
int count = 0;
struct dataTasks results;
// the setup function runs once when you press reset or power the board
void setup() {
  // put your setup code here, to run once:
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BUTTON_1, INPUT);
  pinMode(SQUARE_INPUT,INPUT);
  pinMode(DC_INPUT,INPUT);
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  mutex = xSemaphoreCreateMutex(); // initialisation of the semaphore as mutual exclusion, so the initial value is 1
  

  

  
  // Now set up TASKS.
  xTaskCreatePinnedToCore(
    task1
    ,  "task1"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task2
    ,  "task2"
    ,  1024  // Stack size
    ,  NULL
    ,  1 // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    task3
    ,   "task3"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    task4
    ,  "task4"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    task5
    ,  "task5"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    task6
    ,  "task6"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    task7
    ,  "task7"
    ,  1024  // Stack size
    ,  NULL
    ,  1// Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
 xTaskCreatePinnedToCore(
    task8
    ,  "task8"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
 xTaskCreatePinnedToCore(
    task9
    ,  "task9"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void task1(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  
  {
    //Serial.println("Task1");

    digitalWrite(GREEN, HIGH);
    delayMicroseconds(50); // Block pulse up
    digitalWrite(GREEN, LOW); // Block pulse down  
    vTaskDelay(task1_period);  
  }
}
void task2(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {
    //Serial.println("Task2");
   xSemaphoreTake(mutex, portMAX_DELAY);
   results.buttonState = digitalRead(BUTTON_1); //STATE OF THE BUTTON
   xSemaphoreGive(mutex);
   vTaskDelay(task2_period);  
   
  }
}
void task3(void *pvParameters)  // This is a task.
{
  
  (void) pvParameters;
  for (;;)
  {
    //Serial.println("Task3");
    int highInput = pulseIn(SQUARE_INPUT,HIGH);  //HIGH PULSE PERDIOD uS
    int lowInput = pulseIn(SQUARE_INPUT,LOW);     //LOW PULSE PERIOD uS
    int totalInput = highInput + lowInput; //WHOLE SIGNAL PERIOD
    int frequency=1000000/totalInput;  // CALCUL FREQUENCY IN HZ (1/PERIOD(uS)) * 1000000
    xSemaphoreTake(mutex, portMAX_DELAY);
    results.freqResult = frequency;
    xSemaphoreGive(mutex); 
    vTaskDelay(task3_period);
  }
}
void task4(void *pvParameters)  // This is a task.
{
  
  (void) pvParameters;
  for (;;){
  //  Serial.println("Task4");
    xSemaphoreTake(mutex, portMAX_DELAY);
    results.analogValue = analogRead(DC_INPUT); //READ POTENTIOMETER ANALOG VALUE
    xSemaphoreGive(mutex); 
    vTaskDelay(task4_period); 
  }
}

void task5(void *pvParameters)  // This is a task.
{
  
  
 
  (void) pvParameters;
  for (;;)
  {
    int *ptrSend;
  //  Serial.println("Task5");
    int sum = 0;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if(count == 0 || count % 4 == 0) //UPDATE VALUES IN THE ARRAY 
      analogs[0] = results.analogValue;
    else if(count == 1 || count % 4 == 1) //UPDATE VALUES IN THE ARRAY 
      analogs[1] = results.analogValue;
    else if(count == 2 || count % 4 == 2) //UPDATE VALUES IN THE ARRAY 
      analogs[2] = results.analogValue;
    else
      analogs[3] = results.analogValue;
    xSemaphoreGive(mutex);
    
    //Serial.println(count);
   // Serial.println("0:"+String(analogs[0])+"\n,1:"+String(analogs[1])+"\n,2:"+String(analogs[2])+"\n,3:"+String(analogs[3]));
    count++;  // COUNT THE NUMBER OF VALUE ANALOG RETURNED TO GET THE 4 LAST VALUES WITH MODULO 4 CONDITION
  
    for(int i=0;i<4;i++){
      sum += analogs[i]; //SUM THE VALUES OF THE ARRAY
    }
    
    if(count<4)
      results.analogAvg = sum/count;    //CALCULATE AVG IF NUMBER OF VALUE IN THE ARRAY IS UNDER 4
    else
      results.analogAvg =  sum/4;      //CALCULATE AVG IN THE ARRAY
      
    xSemaphoreTake(mutex, portMAX_DELAY);
    ptrSend = &results.analogAvg;
    xSemaphoreGive( mutex );
    xQueueOverwrite( errorQueue, ptrSend ); 
    vTaskDelay(task5_period);
    
  }
  
 
  
}
void task6(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {
      //Serial.println("Task6");
      for(int i=0;i<1000;i++)
        __asm__ __volatile__ ("nop");

      vTaskDelay(task6_period);
    
  }


}


void task7(void *pvParameters)  // This is a task.
{
  
  (void) pvParameters;
  for (;;)
  {
    //Serial.println("Task7");
    int ptrReceive;
    xQueueReceive( errorQueue, &ptrReceive, portMAX_DELAY );
    int error=0;
    if( ptrReceive > 4095/2)
      error = 1;
    xSemaphoreTake(mutex, portMAX_DELAY);
    results.errorCode = error;
    xSemaphoreGive(mutex);
    vTaskDelay(task7_period);
  }
}

void task8(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {
   // Serial.println("Task8");
    xSemaphoreTake(mutex, portMAX_DELAY);
    digitalWrite(RED, results.errorCode);     //IF ERROR LED SWITCH ON
    xSemaphoreGive( mutex );
    vTaskDelay(task8_period);
        
  }
}
void task9(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {
      Serial.println("Task9");
  
    if( results.buttonState == HIGH){
      xSemaphoreTake(mutex, portMAX_DELAY);
 
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      Serial.println( //PRINT RESULTS
        "bs:"+String(results.buttonState)+
        ",F:"+ String(results.freqResult)+ 
        ",analog:"+String(results.analogValue)+ 
        ",avg:"+String(results.analogAvg)+ 
        ",error:"+String(results.errorCode));
        xSemaphoreGive(mutex); // Now free or "Give" the Serial Port for others.
     
      
    }
    vTaskDelay(task9_period);
  }
}
