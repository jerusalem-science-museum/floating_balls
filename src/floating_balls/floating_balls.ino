// Amir_steper_and_tof_24_JULY_2022
// Control steper motor acording to TOF reading. 
// Move up/down ballons exhbit, 30 year museum eneversery exhebition summer 2022 
// arduino Naon, 4988 steper motor module driver (set current to about 0.05A at 12Volt when motor run/stop)
// NEMA 42 motor
// VL53L1X 400cm (prefer with IR cover) TOF sensor 
// see J:\Bloomfield\Exhibits\היסטוריה לקראת שלושים שנה למוזיאון\פיתוח מוצגים אמיר\כדורים עולים ויורדים
// PCB- J:\Amir Design\PCB_Design\EASYEDA JLBPCB\projects\Stepper_Control
// !!! connect to tof routing is blooking so if tof not responce - motor not move. 
// blink internal led after tof responce 
// 
//
//
#include "Adafruit_VL53L1X.h"
// constants

const uint16_t DIRECTION_CHANGE_WAIT_TIME = 300 ; //[ms] time wait/stop move between change direction to avoid loos steps
const uint32_t TIME_FOR_RESET = 180UL * 1000UL ; //[ms] time for reset (pull string up) 

const int16_t MIN_MOVE  = 0 ; //mm, ball movement 
const float MAX_MOVE  = 1300 ; //mm, brass rod is 1850 mm

const int16_t PULSE_PER_TURN  = 200 ; // standars NIMA stepper 1.8deg per pulse 
const float PULLY_CIRCLE  = 251 ; // 251 mm assume pully dia about 80mm - Baruch double pukky - larger side 
// mm per pulse = PULLY_CIRCLE/PULSE_PER_TURN, pulse per mm = PULSE_PER_TURN/PULLY_CIRCLE 

const int16_t RELEASE_BEFORE_RESET = 1300 ; //  ms, disable drive time befour hommoing release string/ball down 
const uint32_t TIME_PER_MEASURE = 100 ; //[ms] but practical measurment time depend also on TOF sensor timing budget (usealy 100)
const uint16_t TIMING_BUDGET = 100 ; // // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!

const int16_t MINIMUM_NO_ERROR_READ  = 10 ; // minimum reading distance value accepted as real (no error/noise) measure 
const int32_t HYSTERESIS = 50 ; // minimum diferent between old and new distance measure need to change 

const uint16_t NORMAL_RATE = 10; //[ms] = 1/motor_speed [in pulses per sec] the smaler the faster prefer no less the 2 (500 pulse per sec)
const uint16_t RESET_RATE = 15; //[ms] = 1/motor_speed [in pulses per sec]

const float MIN_MEASURE_DISTANCE  = 500 ; //(500 for exhibit, 50 for test) mm, expected minimum sensor read - high person/hand
const int16_t MAX_MEASURE_DISTANCE  = 2500 ; //mm, expected maximum sensor read  - floor level 
const int16_t MIN_PULSES  = PULSE_PER_TURN*(MIN_MOVE/PULLY_CIRCLE) ; //pulse 
const int16_t MAX_PULSES  = PULSE_PER_TURN*(MAX_MOVE/PULLY_CIRCLE) ; //pulses  
const uint32_t PULSE_FOR_RESET  = MAX_PULSES;//100 ; //1.2*MAX_PULSES ; //

const int16_t TOF_START_ON_TIME  = 50 ; // ms, on led blink if TOF  responce  
const int16_t TOF_START_OFF_TIME  = 30 ; // ms 
const int16_t TOF_START_NO_OF_BLINKS  = 5 ; // number of blinks after tof rexponce 

// global variable 
// positive value (positions...) - move forward, Direction = 1-HIGH, release the string- ball down
// negative value - pull the string back (Direction = 0-LOW)
bool Last_Direction_Move = false ;// asuming first direction is pull back = 0 
uint32_t Time_Counter = 0; // counting time between measurments 
uint32_t Reset_Time_Counter = 0; // counting time between rest/homming (pull string up)
int32_t Current_Position = 0; 
int32_t Current_Target_Position = 0; 
int32_t New_Target_Position = 0; 
int16_t Distance = 0;// need to be global (and keep last reading) as read_distance return new
int16_t Old_Distance = 0;// former read need to be global (and keep last reading) as read_distance return very low value
//bool Internal_Led_Statuse = false;// for testin why code stop responce - change every measure (about 100ms)

// The stepper pins
#define STEPPER_EN_PIN 8 // invert - Hige is disable, low enable 
#define STEPPER_DIR_PIN 7 // High (1) is forward - move the string down , Low (0) is backward - pull the string up  
#define STEPPER_STEP_PIN 6 // 1us (min) positive pulse per one step 

#define INTERNAL_LED 13 // 
// for test only 

// TOF pin
#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
//=========================
void blink_internal_led(int16_t led_on,int16_t led_off,int16_t no_off_blinks ) // blink 
  {
  for (int i = 0; i < no_off_blinks; i++)
    {
    digitalWrite(INTERNAL_LED, HIGH);
    delay(led_on);
    digitalWrite(INTERNAL_LED, LOW);
    delay(led_off);
    }
  }
  //-----------------------------
 
  //-----------------------------
void one_step()
  {
  digitalWrite(STEPPER_STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEPPER_STEP_PIN, LOW);
  delayMicroseconds(2);
  }
//--------------------------------------  
void reset_motor(uint16_t rate ,uint32_t num_of_pulses, byte spin_direction) // blooking function !
  { 
//    Serial.println("=============================== NOW RESETING =============================");
    digitalWrite(STEPPER_EN_PIN, HIGH);  // (invert pin) - release motor = string/wheight fall down
    delay (RELEASE_BEFORE_RESET); // 
    digitalWrite(STEPPER_EN_PIN, LOW);  // (invert pin) - active motor
    digitalWrite(STEPPER_DIR_PIN, spin_direction);  
    delay(1);//stable 
  for (uint32_t i = 0 ; i < num_of_pulses ; i++) 
    {
      one_step();
      delay (rate);
    }
  Current_Position = 0;    
  dealy (DIRECTION_CHANGE_WAIT_TIME); // avoid instant move 
}
//-----------------------------------------------
void start_connection_with_sensor()
{
  delay (10);
  Wire.begin();
  delay (10);
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
  delay (10);
  vl53.begin(0x29, &Wire); // with noisy power suppy - code stop here !!!
  vl53.startRanging();
  blink_internal_led(TOF_START_ON_TIME,TOF_START_ON_TIME, TOF_START_NO_OF_BLINKS); // 
  vl53.setTimingBudget(TIMING_BUDGET); //usealy 100
 }
//--------------------------------
void stop_connection_with_sensor()
{
  vl53.stopRanging();
  Wire.end();     
}
//----------------------------------------------
void setup()
{  
  Serial.begin(115200);
  Serial.println("start");   
  pinMode(INTERNAL_LED , OUTPUT);// 
  digitalWrite(INTERNAL_LED,  LOW);  // 

  //next leds for tst only not in the pcb
  pinMode(STEPPER_EN_PIN, OUTPUT);// invert !
  digitalWrite(STEPPER_EN_PIN, HIGH);  // diseable 
  pinMode(STEPPER_DIR_PIN, OUTPUT);//  !
  digitalWrite(STEPPER_DIR_PIN,  LOW);  // diseable 
  pinMode(STEPPER_STEP_PIN, OUTPUT);// invert !
  digitalWrite(STEPPER_STEP_PIN,  LOW);  // diseable 

  digitalWrite(INTERNAL_LED,  LOW);  // 
  Time_Counter = millis();// reset counter 
  Reset_Time_Counter = millis();// reset counter 
  
  start_connection_with_sensor();
  reset_motor(RESET_RATE , PULSE_FOR_RESET, 0 ); //
}
//====================
void loop()
{
  // check time for reset 
   if (millis() >= (Reset_Time_Counter + TIME_FOR_RESET)) // reset/homming system (pull string up) 
    {
    stop_connection_with_sensor();
    reset_motor(RESET_RATE , PULSE_FOR_RESET, 0 ); //
    start_connection_with_sensor();
    Reset_Time_Counter = millis();// reset reset timer 
    }
//----------------------------------------
// check time for new distance measure   
   if (millis() >=( Time_Counter + TIME_PER_MEASURE)) // read sensors 
    {
    Distance = vl53.distance();
    if (Distance <= MINIMUM_NO_ERROR_READ) {Distance = Old_Distance;}//problem with reading - ignore very low value 
    Old_Distance = Distance ; // update (or keep last if Distance was -1)
    int16_t tamp_in = Distance;
    New_Target_Position = map(tamp_in, MIN_MEASURE_DISTANCE, MAX_MEASURE_DISTANCE, MIN_PULSES, MAX_PULSES);
    if (New_Target_Position < MIN_PULSES) New_Target_Position = MIN_PULSES ; // make sure in range (maby there is other map option) 
    if (New_Target_Position > MAX_PULSES) New_Target_Position = MAX_PULSES ;
    if ((New_Target_Position -  Current_Target_Position) >= HYSTERESIS || (Current_Target_Position - New_Target_Position) >= HYSTERESIS) 
      {Current_Target_Position = New_Target_Position; }// update target       
    Time_Counter = millis();// reset measure time counter 
    }
// compare position to current position and pulse acording     
    if (Current_Target_Position > Current_Position)// move one step forward lower ball 
    { 
      if(Last_Direction_Move == false) { 
        delay (DIRECTION_CHANGE_WAIT_TIME);//  last direction was up so wait/stop befour change direction 
        Last_Direction_Move = true ; // now movin down
        }
      digitalWrite(STEPPER_DIR_PIN,  HIGH);  // move forward ! 
      digitalWrite(INTERNAL_LED,  HIGH);  // iindicate lower(push) ball
      delayMicroseconds(1);// stable 
      one_step();
      Current_Position++;
    }
    if (Current_Target_Position < Current_Position)// move one step back 
      {
       if(Last_Direction_Move == true) { 
       delay (DIRECTION_CHANGE_WAIT_TIME);//  last direction was down so wait/stop befour change direction 
       Last_Direction_Move = false ; // now moving up 
        }
      digitalWrite(STEPPER_DIR_PIN,  LOW);  // move backward ! 
      digitalWrite(INTERNAL_LED,  LOW);  // indicate pull ball
      delayMicroseconds(1);// stable 
      one_step();
      Current_Position--;
    }
    Serial.print("Current_Position:  ");   
    Serial.print(Current_Position);   
    Serial.print(",   Current_Target_Position:  ");   
    Serial.print(Current_Target_Position);   
    Serial.print(",   Last_Direction_Move:  ");   
    Serial.println(Last_Direction_Move);
    delay(NORMAL_RATE) ; 
}
  
  
   
