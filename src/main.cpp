/*
===============================CHOPPER VIDEO TESTER====================================================
Measure video and chopper usisng ESP32
conect the chopper to pin 35 and video pin 22
the baudas is 115200-8-None-1
Rudimentary sincro when the signal chopper is High; flag is in false
the task comunication no send data;
in this way we allow time for the communication task to send the last calculated values when it reaches its second

TODO: improve synchronism; store all timing data in memory.
only works on 1 axis, it will have to be improved for 2 or 3 axes.
This way you will be able to identify each 1 channel.
Expose all the data through wifi using the parallelism of the 2 integrated cores,



packages:
    name=ESP32 Arduino
    version=2.0.14

    set(min_supported_idf_version "4.4.0")
    set(max_supported_idf_version "4.4.99")
================================================================================================
*/

#include <Arduino.h>

void ICACHE_RAM_ATTR falling_chopper();
void ICACHE_RAM_ATTR rising_chopper();
void ICACHE_RAM_ATTR falling_video();
void ICACHE_RAM_ATTR rising_video();

#define PIN_CHOPPER 35  // PIN where the PWM singal arrives
#define PIN_VIDEO 22  // PIN where the PWM singal arrives

volatile uint64_t StartValue_video = 0;    // First interrupt value in video pin (timer value when rising video)            
volatile uint64_t PeriodCount_video = 0;   //acumulate the multiples (n*times) from the start_video rising to calculate freq
volatile uint64_t StopValue_video=0;        //timer value when falling video
float Freq_video;   
volatile uint64_t StartValue_chopper = 0;   // First interrupt value in chopper pin          
volatile uint64_t PeriodCount_chopper = 0;   //acumulate the multiples from the Star_ chopper rising to calculate freq
volatile uint64_t StopValue_chopper=0;    //timer value when falling video
float Freq_chopper;                          
volatile uint64_t  pulse_width_chopper = 0; 
volatile uint64_t  pulse_width_video = 0; 
volatile uint64_t vidchop_t1 =0;   //time betewen rissing video - rissing chopper  (chopper - video)
volatile uint64_t vidchop_t2 =0;   //time bettwen falling chpper - falling video  (video - chopper)

portMUX_TYPE mux0 = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?
hw_timer_t * timer0 = NULL;                        // pointer to a variable of type hw_timer_t
volatile bool flag = false;                     //rudimentary sincronization when there is Video siganal

void clearValues(){
  StartValue_video = 0;                
  PeriodCount_video = 0; 
  StopValue_video=0;
  StartValue_chopper = 0;                
  PeriodCount_chopper = 0; 
  StopValue_chopper=0;                         
  pulse_width_chopper = 0; 
  pulse_width_video = 0; 
  vidchop_t1 =0;
  vidchop_t2 =0;
}

//=================================Measurin chopper time============================================================

void rising_chopper() {
  portENTER_CRITICAL_ISR(&mux0);
  flag=false;
  attachInterrupt(digitalPinToInterrupt(PIN_CHOPPER), &falling_chopper, FALLING);  // when PIN goes LOW, call falling()
  uint64_t TempVal = timerRead(timer0);            // value of timer at interrupt
  PeriodCount_chopper = TempVal - StartValue_chopper;             // period count between rising edges
  StartValue_chopper = TempVal; 
  vidchop_t1 = StartValue_chopper - StartValue_video;                           // puts latest reading as start for next calculation
  portEXIT_CRITICAL_ISR(&mux0);
}
 
void falling_chopper() {
  portENTER_CRITICAL_ISR(&mux0);
  attachInterrupt(digitalPinToInterrupt(PIN_CHOPPER), &rising_chopper, RISING);  // when PIN goes HIGH, call rising()
  uint64_t TempVal = timerRead(timer0);
  StopValue_chopper = TempVal;
  pulse_width_chopper = StopValue_chopper  - StartValue_chopper;
  flag=true;
  portEXIT_CRITICAL_ISR(&mux0);
}

//================================Measure video time==============================================================

void rising_video() {
  portENTER_CRITICAL_ISR(&mux0);
  //flag=false;
  attachInterrupt(digitalPinToInterrupt(PIN_VIDEO), &falling_video, FALLING);  // when PIN goes LOW, call falling()
  uint64_t TempVal = timerRead(timer0);            // value of timer at interrupt
  PeriodCount_video = TempVal - StartValue_video;             // period count between rising edges
  StartValue_video = TempVal; 
  portEXIT_CRITICAL_ISR(&mux0);
}
 
void falling_video() {
  portENTER_CRITICAL_ISR(&mux0);
  attachInterrupt(digitalPinToInterrupt(PIN_VIDEO), &rising_video, RISING);  // when PIN goes HIGH, call rising()
  uint64_t TempVal = timerRead(timer0); 
  StopValue_video = TempVal;
  pulse_width_video = StopValue_video - StartValue_video;
  vidchop_t2 = StopValue_video-StopValue_chopper;
  //flag=true;
  portEXIT_CRITICAL_ISR(&mux0);
}


//===========================Task Comunication=========================================================

void vTask(void *pvParameters){
  while(true)
  {  
    float t1 = vidchop_t1*25/1000.0;    //calculate in microseconds
    float t2 = vidchop_t2*25/1000.0;
    float video = pulse_width_video*25/1000.0;    //calculate in microseconds
    float chopper = pulse_width_chopper*25/1000.0;
     Freq_video = 40000000.00 / PeriodCount_video;   // calculate frequency 
     Freq_chopper = 40000000.00 / PeriodCount_chopper;   
    if (flag){                      
    Serial.print("Fvid : "); Serial.print(Freq_video, 0);
    Serial.print("  Fchopp  : "); Serial.print(Freq_chopper, 0);
    Serial.print("  Tup : "); Serial.print(t1,2);
    Serial.print("   TDown : "); Serial.print(t2,2);
    Serial.print("  tvid : "); Serial.print(video,2);
    Serial.print("  tchop : "); Serial.println(chopper,2);
    timerRestart(timer0);
    clearValues();
    }
    vTaskDelay( pdMS_TO_TICKS( 1000) );
  }
  vTaskDelete(NULL);

}

//===============================Config-Init=====================================================

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_CHOPPER, INPUT_PULLUP);   // sets pin as input
  pinMode(PIN_VIDEO, INPUT_PULLUP);                                           
  xTaskCreate(vTask, "Task1",1000,NULL,1,NULL);
  timer0 = timerBegin(0, 2, true); // configure timer                                 
  // 0 = first timer
  // 2 is prescaler so 80 MHZ divided by 2 = 40 MHZ signal       //25ns  of each count
  // true - counts up
  timerStart(timer0);
  attachInterrupt(digitalPinToInterrupt(PIN_CHOPPER), &rising_chopper, RISING);  // when PIN goes HIGH, call rising()
  attachInterrupt(digitalPinToInterrupt(PIN_VIDEO), &rising_video, RISING);  // when PIN goes HIGH, call rising()                                                
}

void loop(){}

