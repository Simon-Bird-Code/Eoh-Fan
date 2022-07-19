/*
Project : Eoh Fan Control
Version : 0v40
Author  : Simon J Bird
Date    : 2/06/2022
Contact : SimonBird.Code@Gmail.com
Final Release Circuit : N/A
*/


/*
 * BREIF
 Eoh Fan Control - Technical Description V1.pdf
 */

#define PROJECT "Eoh Fan Control"
#define VERSIONNUMBER 040

//************* PROJECT DEFINES  **************//  -- NOT NORMALLY HERE

#define CORE_THRESHOLD 25
#define EDGE_THRESHOLD 30
#define CORE_EDGE_THRESHOLD 2
#define CORE_OUTSIDE_THRESHOLD 10

#define NUMBER_OF_CONDITIONS 4

#define FAN_MIN_RUNNING_TIME 6000  		// millisecs

#define KELVIN_OFFSET 273.15    		// everything in kelvin

#define SERIAL_REPORT_TIME 2000     	// millisecs

//********************************  INCLUDES  *******************************//
//********                                                          *********//

//#include <EEPROM.h>  // not used in this code 
#include <math.h>

//************************** HARDWARE CONNECTIONS ***************************//
//********                                                          *********//

//***************** PIN DEFINES ***************//
// ATmega - docs.arduino.cc/hacking/hardware/PinMapping168
// usefull if using standalone chipset and custom PCB...

                        //  physical Pin 1  |   Reset     |  Do Not Use
#define UNUSED0     0   //  physical Pin 2  |   RX        |  + GPIO - Not Used 
#define UNUSED1     1   //  physical Pin 3  |   TX        |  + GPIO - Not Used
#define UNUSED2     2   //  physical Pin 4  |   GPIO      |  Not Used
#define LEDSTATUS   3   //  physical Pin 5  |   PWM       |  not used in this code
#define LEDPOWER    4   //  physical Pin 6  |   GPIO      |  not used in this code
                        //  physical Pin 7  |   POWERVCC  |
                        //  physical Pin 8  |   POWERGRD1 |
                        //  physical Pin 9  |   Crystal   |  guessing standard uno 16
                        //  physical Pin 10 |   Crystal   |  guessing standard uno 16            
#define CH3_FAN     5   //  physical Pin 11 |   GPIO      |  Relay digital - Center fans (*8)
#define CH2_FAN     6   //  physical Pin 12 |   GPIO      |  Relay digital - left side fans (*5)
#define CH1_FAN     7   //  physical Pin 13 |   GPIO      |  Relay digital - right side fans (*5)
#define UNUSED3     8   //  physical Pin 14 |   GPIO      |
#define UNUSED4     9   //  physical Pin 15 |   PWM       |
#define UNUSED5     10  //  physical Pin 16 |   PWM       |  BoardLED
#define UNUSED6     11  //  physical Pin 17 |   GPIO      |
#define UNUSED7     12  //  physical Pin 18 |   GPIO      |
#define UNUSED8     13  //  physical Pin 19 |   GPIO      |
                        //  physical Pin 20 |   POWERAVCC |
                        //  physical Pin 21 |   POWERVCC2 | 
                        //  physical Pin 22 |   POWERGRD2 | 
#define TEMP_1      14  //  physical Pin 23 |   A0        |  Thermistor 1 - mod A center bottom
#define TEMP_2      15  //  physical Pin 24 |   A1        |  Thermistor 2 - mod b center middle
#define TEMP_3      16  //  physical Pin 25 |   A2        |  Thermistor 3 - mod c center top
#define TEMP_4      17  //  physical Pin 26 |   A3        |  Thermistor 4 - mod b side offset 
#define TEMP_5      18  //  physical Pin 27 |   A4        |  Thermistor 5 - mod c side offset
#define TEMP_6      19  //  physical Pin 28 |   A5        |  Thermistor 6 - unit outside temp / ambient

//*********** THERMISTER PROPERTIES ***********//
// Vishay NTCALUG01A103F NTC thermistor 10Kohm  
// EXAMPLE ONLY - but fairly sure one specified

#define THERMISTOR_RESISTANCE       10000   // resistance at 
#define THERMISTOR_CALIBRATION_TEMP 25      // degrees c
#define PAIRED_RESISTOR             10000   // series resister value
#define BETA_COEFF                  3984    // bata coefficent via datasheet

#define THERMISTOR_REACTION_TIME_MIN 6      // seconds
#define THERMISTOR_REACTION_TIME_MAX 14     // seconds

//**************** CHIP SPECS *****************//

#define CHIPSET "ATmega"                	// arduino uno board
#define HARDWARESPEED 16                  	// MHz, has crystal installed on uno hardware ??
#define ADC_RESOLUTION 1024               	// 12 bit resolution
//#define BUTTONDEFAULTHIGH true            // high for default , pull up

//********** EEPROM memory locations **********//

#define PROGRAMLENGTH           6276 

#define DEBUGMEMBANK1           10000  		// USER SPACE
#define DEBUGMEMBANK2           10201  		// USER SPACE

#define SECURITYCOUNTMEMBANK1   12001  		// USER SPACE
#define SECURITYKEYMEMBANK1     13001  		// USER SPACE

//************ CONNECTED HARDWARE *************//

//#define MAX_BUTTONS   4
//#define MAX_LEDS      4
#define MAX_SENSORS   6           // thermistors
#define MAX_RELAYS    3           // fan relay board - arduino UNO Relay Sheild 


//**************************** SOFTWARE DEFINES *****************************//
//********                                                          *********//

//****************** MODES ********************//

//#define DEBUG       false
//#define DEBUG_FULL  false
//#define DEBUG_SAFE  false

//****************** COMMS ********************//

#define COMMS_BAUD 9600

//************* PROJECT DEFINES  **************//
//*************** NOW AT TOP ******************//

//#define CORE_THRESHOLD 25
//#define EDGE_THRESHOLD 30
//#define CORE_EDGE_THRESHOLD 2
//#define CORE_OUTSIDE_THRESHOLD 10

//#define NUMBER_OF_CONDITIONS 4

//#define FAN_MIN_RUNNING_TIME 6000  		// millisecs

//#define KELVIN_OFFSET 273.15    		// everything in kelvin

//#define SERIAL_REPORT_TIME 2000     	// millisecs

//****************** TIMINGS ******************//

#define LOOP_TIME 1000               // millisecs - /// NEEDS CALCULATING

//***************************** GLOBAL VARIABLES ****************************//
//********                                                          *********//


bool condition  [NUMBER_OF_CONDITIONS];
// 0- cores hotter then side , 1- cores hotter then ambient
// 2- cores hotter then specified , 3- edges hotter then specified

bool running_conditions[NUMBER_OF_CONDITIONS];

long int timers [NUMBER_OF_CONDITIONS];

unsigned long serial_output_timer=0;

float T1,T2,T3,T4,T5,T6;            // ACTUAL TEMPS
bool CH1 , CH2 , CH3;               // RELAY CHANNELS , 8 fans , 5 fans and 5 fans respectivly


unsigned long time_now = 0 ;        // used to store updated time
 

//*************************** FUNCTION PROTOTYPES ***************************//
//********                                                          *********//

void setup();
void loop();

void hardware_setup              ( void );
void software_setup              ( void );

void read_all_sensors            ( void );
float resistance_over_thermistor ( int adc_in );
float conversion_steinhart       ( int ADC_value_in );

void loop_time_check             ( void ) ;
void start_condition_timer       ( int condition );
void stop_condition_timer        ( int condition );
void adjust_timers               ( void );


void clear_conditions            ( void );
void check_for_conditions        ( void ) ;
void action_condition            ( void );

void start_condition             ( int condition_number );
void restart_condition_timer     ( int condition_number ) ;
void stop_condition              ( int condition_number );
void stop_all_condition          ( int condition_number );

void fans                        ( int condition_in , bool state_in );
void serial_report               ( void );
void loop_fan_control            ( void );

void read_all_sensors_test       ( void );

void setup_testbed               ( void );
void loop_testbed                ( void );

//******************************* FULL FUNCTIONS ****************************//
//********                                                          *********//


//******************** HARDWARE SETUP ********************//


// iniz all pin states 
void hardware_setup ( void )
{
  /// set up the wired pins, as this is fixed at every boot
  pinMode(LEDPOWER,OUTPUT);
  pinMode(LEDSTATUS,OUTPUT);
    
  pinMode(TEMP_1,INPUT);
  pinMode(TEMP_2,INPUT);
  pinMode(TEMP_3,INPUT);
  pinMode(TEMP_4,INPUT);
  pinMode(TEMP_5,INPUT);
  pinMode(TEMP_6,INPUT);
    
  pinMode(UNUSED5,OUTPUT);  //#define UNUSED5     10  //  physical Pin 16 |   PWM       |  BoardLED
    
  pinMode(CH3_FAN,OUTPUT);
  pinMode(CH2_FAN,OUTPUT);
  pinMode(CH1_FAN,OUTPUT);
}


//******************** SOFTWARE SETUP ********************//


// iniz all code elements , memory etc -- software side - can be redone in run time 
void software_setup ( void ) 
{   
  Serial.begin(COMMS_BAUD);
  
  for(int i=0;i<NUMBER_OF_CONDITIONS;i++)
    {
      condition[i]=false;
      running_conditions[i]=false;
      timers[i]=0;
    }
  
  T1=T2=T3=T4=T5=T6=0;   
  
  CH1=CH2=CH3=false;  
  
  time_now=millis();
  
  serial_output_timer=SERIAL_REPORT_TIME;
}


//********************** READ INPUTS *********************//


/// reads ADC and converts to temp T1-T6
void read_all_sensors ( void )
{
  // do all read as takes time for ADC = delta small
  uint16_t value_1=analogRead (TEMP_1);
  uint16_t value_2=analogRead (TEMP_2);
  uint16_t value_3=analogRead (TEMP_3);
  uint16_t value_4=analogRead (TEMP_4);
  uint16_t value_5=analogRead (TEMP_5);
  uint16_t value_6=analogRead (TEMP_6);
  // then do all conversions
  T1 = conversion_steinhart ( value_1 );
  T2 = conversion_steinhart ( value_2 );
  T3 = conversion_steinhart ( value_3 );
  T4 = conversion_steinhart ( value_4 );
  T5 = conversion_steinhart ( value_5 );
  T6 = conversion_steinhart ( value_6 ); 
}


//********************* CALCULATIONS *********************//


float resistance_over_thermistor ( int adc_in )
{ 
    ///   vf/vt=rt/rf
	///   vs - vt = vf 
	///   vs - vt / vt = rt / rf 
    ///  (vs/vt) - 1 = rt / rf  
	///   vs/vt = max / (max - adc) 
    ///   so ((max/(max-adc))-1)*rf = rt  ----  tested and works
     
    float ret; 
    ret = (  ( (float)ADC_RESOLUTION / ( (float)ADC_RESOLUTION-(float)adc_in ) ) - 1 ) * (float)PAIRED_RESISTOR ;
    return ret;
}

// verstion 1 
float conversion_steinhart ( int ADC_value_in )
{
  /// Formula =  R = R0 * exp ( B * ( 1/T - 1/T0 ) ) 
  /// REARRANGE :-
  /// R/R0
  /// log ( R/R0 )
  /// /B
  /// + 1/T0
  /// 1 / ( 1/T ) = T 
  
  float running_calc=0.0;
  float instance_temp=0.0;
   
  float kelvin_offset = 273.15;

  float Thermistor_resistance_measured =  resistance_over_thermistor (ADC_value_in );    

  running_calc = Thermistor_resistance_measured / (float)THERMISTOR_RESISTANCE; // R/R0
  
  running_calc = log(running_calc);                                 			// inverse exp

  running_calc /= (float)BETA_COEFF;                                  			// 1/B * ln(R/R0)

  running_calc += 1.0 / ((float)THERMISTOR_CALIBRATION_TEMP + kelvin_offset);   // + (1/T0)

  running_calc = 1.0 / running_calc;                                			// invert

  running_calc -= kelvin_offset;                                    			// convert to c

  instance_temp=running_calc;
  
  return instance_temp;
}


//************************ TIMERS ***********************//

// this function protects against rollover as unsigned int - after 50 days there is a rollover
void loop_time_check( void )        
{ 
  if ((unsigned long ) ( millis()- time_now) > LOOP_TIME ) 
  {
    adjust_timers();time_now = millis();
  }
 }

void start_condition_timer ( int condition )    /// could have cool down time for each fan config, and each temp difference
{
    timers[condition] = FAN_MIN_RUNNING_TIME; 
}

void stop_condition_timer ( int condition )
{
    timers[condition] = 0;
}

void adjust_timers ( void )           /// adjust timers as per loop conditions and time allowed by loop
{
  for(int i=0;i<NUMBER_OF_CONDITIONS;i++)
  {
    if(timers[i]>0)
    {  
      timers[i] = timers[i] - LOOP_TIME;
    }
    if(timers[i]<0)    {    timers[i] = 0;    }
  }
  serial_output_timer=serial_output_timer - LOOP_TIME;
}


//********************* STATE MACHINES ******************//


void clear_conditions ( void )
{
  for(int i=0;i<NUMBER_OF_CONDITIONS;i++){condition[i] = false;}
}


void check_for_conditions ( void ) 
{ 
  // this works out what condition the system is in, depending on the current temperature readings

  
      if(  T1  > ( T4+CORE_EDGE_THRESHOLD )  )      { condition[0] = true; }
      if(  T2  > ( T4+CORE_EDGE_THRESHOLD )  )      { condition[0] = true; }
      if(  T3  > ( T4+CORE_EDGE_THRESHOLD )  )      { condition[0] = true; }

      if(  T1  > ( T5+CORE_EDGE_THRESHOLD )  )      { condition[0] = true; }
      if(  T2  > ( T5+CORE_EDGE_THRESHOLD )  )      { condition[0] = true; }
      if(  T3  > ( T5+CORE_EDGE_THRESHOLD )  )      { condition[0] = true; }

      if( ( T1 ) > (T6 + CORE_OUTSIDE_THRESHOLD) )  { condition[1] = true;}
      if( ( T2 ) > (T6 + CORE_OUTSIDE_THRESHOLD) )  { condition[1] = true;}
      if( ( T3 ) > (T6 + CORE_OUTSIDE_THRESHOLD) )  { condition[1] = true;}

      if( ( T1 ) > CORE_THRESHOLD )                 { condition[2] = true;}
      if( ( T2 ) > CORE_THRESHOLD )                 { condition[2] = true;}
      if( ( T3 ) > CORE_THRESHOLD )                 { condition[2] = true;}

      if( ( T4 ) > EDGE_THRESHOLD )                 { condition[3] = true;}
      if( ( T5 ) > EDGE_THRESHOLD )                 { condition[3] = true;}
}


void action_condition  ( void )
{
    for(int i=0;i<NUMBER_OF_CONDITIONS;i++)
    {
        if (condition[i]  &&  !running_conditions[i] )  { if (timers[i]<=0) { start_condition(i);         } }
        if (condition[i]  &&  running_conditions[i]  )  { if (timers[i]<=0) { restart_condition_timer(i); } }
        if (!condition[i] &&  running_conditions[i]  )  { if (timers[i]<=0) { stop_condition(i);          } }
    //  if (!condition[i] &&  running_conditions[i]  )  { if (timers[i]>0)  { stop_all_condition(i);      } }
	// this last is the OR condition no longer, however, would flicker so bad idea
    }
  
}

void start_condition          ( int condition_number )
{  
    start_condition_timer ( condition_number );
    running_conditions[condition_number] = true;
    fans ( condition_number , true ); 
}

void restart_condition_timer  ( int condition_number ) 
{
    start_condition_timer ( condition_number );
}

void stop_condition           ( int condition_number ) 
{
    running_conditions[condition_number] = false;
    fans ( condition_number , false ); 
}

void stop_all_condition       ( int condition_number )
{   
   running_conditions[condition_number] = false;
   fans ( condition_number , false );
   stop_condition_timer( condition_number );    
}       

//********************* SET OUTPUTS *********************//


void fans ( int condition_in , bool state_in )
{
  if(condition_in < 3  )  
  {
    digitalWrite(CH1_FAN,state_in);
    CH1=state_in; 
  }
  if(condition_in == 3 ) 
  { 
    digitalWrite(CH2_FAN,state_in);
    CH2=state_in;
    digitalWrite(CH3_FAN,state_in);
    CH3=state_in; 
  }
}

// basic reporting of values - as specified, however a csv would be better, and maybe more info
void serial_report ( void )  // type 1 
{
  if(serial_output_timer<=0)
    {
      Serial.print(" T1  = ");   Serial.print(T1);    Serial.print("\n");
      Serial.print(" T2  = ");   Serial.print(T2);    Serial.print("\n");
      Serial.print(" T3  = ");   Serial.print(T3);    Serial.print("\n");
      Serial.print(" T4  = ");   Serial.print(T4);    Serial.print("\n");
      Serial.print(" T5  = ");   Serial.print(T5);    Serial.print("\n");
      Serial.print(" T6  = ");   Serial.print(T6);    Serial.print("\n");
      Serial.print(" CH1 = ");   Serial.print(CH1);   Serial.print("\n");
      Serial.print(" CH2 = ");   Serial.print(CH2);   Serial.print("\n");
      Serial.print(" CH3 = ");   Serial.print(CH3);   Serial.print("\n");
      Serial.print("\n");
      
      serial_output_timer = SERIAL_REPORT_TIME; // reset timer
    }
 }



// ********************** MAIN LOOP *********************//

void loop_fan_control ( void )
{
  read_all_sensors_test();         // read sensors do calc and load t1-t6
  clear_conditions();
  check_for_conditions();
  action_condition();
  serial_report();
  loop_time_check();
 }


//***************** ARDUINO FUNCTIONS *******************//


void setup()          // on power up
{
  hardware_setup();
  software_setup();
  delay(1000);  
}


 
 void loop(void)
 {
     loop_fan_control();
 }


//****************** BASIC PROGRAM TEST ****************//


/// 1=49 6=54
/// a=97 f=102
/// make it t up and down, instead of read sensors


void read_all_sensors_test( void )
{
  int incomingByte = 0;
  if ( Serial.available() > 0 )
  {
    incomingByte = Serial.read();
    if(incomingByte == 13){}      //
    if(incomingByte == 10){}      // enter and carrage return windows system

    if(incomingByte == 49){T1=T1+1.0;}      // 1
    if(incomingByte == 50){T2=T2+1.0;}      // 2
    if(incomingByte == 51){T3=T3+1.0;}      // 3
    if(incomingByte == 52){T4=T4+1.0;}      // 4
    if(incomingByte == 53){T5=T5+1.0;}      // 5 
    if(incomingByte == 54){T6=T6+1.0;}      // 6
    
    if(incomingByte == 97){T1=T1-1.0;}      // a
    if(incomingByte == 98){T2=T2-1.0;}      // b
    if(incomingByte == 99){T3=T3-1.0;}      // c
    if(incomingByte == 100){T4=T4-1.0;}     // d
    if(incomingByte == 101){T5=T5-1.0;}     // e
    if(incomingByte == 102){T6=T6-1.0;}     // f
   }
}





