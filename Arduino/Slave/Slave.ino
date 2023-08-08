#include <PID_v1.h>
#include <Arduino.h>
#include "Thread.h"

// *** Thread ***
// data structure of the thread
typedef struct data_type {
      String title;
      long   pause;
}
      data_type;
      
// state structure of the thread
typedef struct state_type {
      char stack[500];
      data_type data;
}
    state_type;

// static allocate the state variables
state_type statePID;

// sync barier for sharing the printer
void * mutex_serial = NULL;

__attribute__((OS_task)) void thread_loop(void);
// *** end of thread

//*** PID ***
//Define Variables we'll be connecting to
double Setpoint0, Input0, Output0;
double Setpoint1, Input1, Output1;

//Define the aggressive and conservative Tuning Parameters
double aggKp0=1, aggKi0=0.02/*0.00005*/, aggKd0=0.1/*5*/;
double consKp0=0, consKi0=0, consKd0=0;

double aggKp1=30, aggKi1=1, aggKd1=0.1;
double consKp1=0, consKi1=0, consKd1=0;

//Specify the links and initial tuning parameters
PID myPID0(&Input0, &Output0, &Setpoint0, aggKp0, aggKi0, aggKd0, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint1, aggKp1, aggKi1, aggKd1, DIRECT);
//*** end of PID ***

int pin_P0In = A0;
int pin_P1In = A1;
int pin_switchP0 = 3;
int pin_switchP1 = 4;

float val0, val1;
String wStr, rStr;
byte wBuf[2], rBuf[2];
int started_flag = 0;
float val0_offset = 1.0;
float val1_offset = 1.0;

float P0Value = 0.0;
float P1Value = 0.0;
int PIDStarting = 0;

int flip0 = 0;
int flip1 = 0;
int flip0_ = 20;
int flip1_ = 20;

float pressure0 = 0.0;
float pressure1 = 0.0;

void setup() {
  // analogReference (EXTERNAL) ;
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  
  //turn the PID on
  myPID0.SetMode(AUTOMATIC);
  //turn the PID on
  myPID1.SetMode(AUTOMATIC);

  pinMode(pin_switchP0, OUTPUT);
  pinMode(pin_switchP1, OUTPUT);
  
  digitalWrite(pin_switchP1, HIGH); // Close  
  digitalWrite(pin_switchP0, HIGH); // Close
    
  // need to enable for spawn call
    schedule();
  
  // spawn the flow PID
    statePID.data.title = "PID";
    statePID.data.pause = 50;
    spawn(&statePID.data, &thread_loop);
}

//https://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int PIDCompute(float diff, PID *myPID, double *Output, double aggKp, double aggKi, double aggKd, double consKp, double consKi, double consKd)
{  
    if (abs(diff > 0.00))
    {
       //we're far from setpoint, use aggressive tuning parameters
       myPID->SetTunings(aggKp, aggKi, aggKd);
    }
    
    else if (abs(diff <= 0.05))
    {
      //we're close to setpoint, use conservative tuning parameters
      myPID->SetTunings(consKp, consKi, consKd);      
    }
    else
    {
      //myPID->SetTunings(aggKp, aggKi, aggKd);
    }
    //myPID->SetTunings(aggKp, aggKi, aggKd);
    myPID->Compute();
}

void PIDAdjust(float diff, PID *pMyPID, double *pOutput, double setValue, int *pFlip, int *pFlip_, int pin, double aggKp, double aggKi, double aggKd, double consKp, double consKi, double consKd)
{  
    PIDCompute(diff, pMyPID, pOutput, aggKp, aggKi, aggKd, consKp, consKi, consKd);
    int diffNeg = 1000;
    /*
    if (diff < 0)
    {
      *pFlip_ = *pOutput > 1 ? diffNeg: int((*pOutput) * diffNeg); 
      digitalWrite(pin, HIGH); // Close   
      // flip0_ = 2; 
      // flip0 = 1;
    }
    else
    {
      *pFlip_ = *pOutput > 1? 1:(10 - int(*pOutput * 10));      
    }  */
    //*pFlip_ = *pOutput > 1? 1:(100 - int(*pOutput * 100));  
    *pFlip_ = *pOutput > 1? 100:(int(*pOutput * 100));  
    if ((*pFlip % 100 < *pFlip_) && (diff > 0) && (setValue != 0))
    {
      // increase pressure
      digitalWrite(pin, LOW); // Open  
      // Serial.println("PID: Open:");
      // PIDPrint("PID increase pressure.");    
    }
    else
    {
      // decrease pressure
      // Serial.println(*pOutput);
      digitalWrite(pin, HIGH); // Close   
    }    
}

/* Reentrant script */
void thread_loop(void)
{  
  flip0 = flip0 + 1;
  flip1 = flip1 + 1;

  if (PIDStarting)
  {    
    Input0 = (double) pressure0;
    Input1 = (double) pressure1;
    
    PIDAdjust(P0Value - pressure0, &myPID0, &Output0, P0Value, &flip0, &flip0_, pin_switchP0, aggKp0, aggKi0, aggKd0, consKp0, consKi0, consKd0);
    PIDAdjust(P1Value - pressure1, &myPID1, &Output1, P1Value, &flip1, &flip1_, pin_switchP1, aggKp1, aggKi1, aggKd1, consKp1, consKi1, consKd1);
  }
  /*
  float diff = P0Value - pressure0;
  if (PIDStarting)
  {    
    PIDCompute(diff, &myPID0, &Output0);
    
    if (diff < 0)
    {
      flip0_ = Output0 > 1 ? 1000: int((Output0) * 1000); 
      // flip0_ = 2; 
      // flip0 = 1;
    }
    else
    {
      flip0_ = Output0 > 1? 1:(10 - int(Output0 * 10));      
    }  
    if (flip0 % flip0_ == 0)
    {
      // increase pressure
      digitalWrite(pin_switchP0, LOW); // Open  
      // Serial.println("PID: Open:");
      // PIDPrint("PID increase pressure.");    
    }
    else
    {
      // decrease pressure
      digitalWrite(pin_switchP0, HIGH); // Close    
    }    
  }*/
  /*
  if (flip0 % 100 == 0)
  {
    // Serial.println("PIDOutputs:" + String(Output0) + ',' + String(Output1));   
    Serial.println("PIDOutputs: Output0:" );     
    Serial.println(Output0 );   
    Serial.println(flip0_);
    Serial.println(flip0);
  }*/
  
    // unblocked delay
  delay(((data_type*)thread)->pause);
}

void averageAnalogReading(int pin0, int pin1, float *pVal0, float *pVal1, bool flag)
{
  if (flag)
  {
    float v0, v1, v2;
    float v0_2, v1_2, v2_2;
  
    v0 = analogRead(pin0);
    v0_2 = analogRead(pin1);
    delay(30);
    v1 = analogRead(pin0);
    v1_2 = analogRead(pin1);
    delay(30);
    v2 = analogRead(pin0);
    v2_2 = analogRead(pin1);
    delay(30);
  
    *pVal0 = (v0 + v1 + v2) / 3.0;
    *pVal1 = (v0_2 + v1_2 + v2_2) / 3.0;    
  }
  else
  {
    *pVal0 = analogRead(pin0);
    *pVal1 = analogRead(pin1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  averageAnalogReading(pin_P0In, pin_P1In, &val0, &val1, false);
  val0 = val0 * val0_offset;
  val1 = val1 * val1_offset;
  
    // calibration
  if (Serial.available() >= 1)
  {
    rStr = Serial.readString();
    rStr.trim();
    if (rStr == "Calibration")
    {      
      val0_offset = String(1024.0 / 5.0 / val0, 3).toFloat();
      val1_offset = String(1024.0 / 5.0 / val1, 3).toFloat();
      
      Serial.write("Calibration done!\r\n");
      Serial.println(String(val0_offset) + ',' + String(val1_offset));
      Serial2.write("Calibration done!\r\n");
      Serial2.println(String(val0_offset) + ',' + String(val1_offset));
    }
    else if (rStr.startsWith("PID") )
    {
      P0Value = getValue(rStr, ':', 1).toFloat();
      Setpoint0 = P0Value;
      P1Value = getValue(rStr, ':', 2).toFloat();
      Setpoint1 = P1Value;
      /*
      String str_pressure = String(P0Value) + ',' + String(P1Value);
      Serial.println( str_pressure );*/
      PIDStarting = 1;
    }
  }
  // end of calibration
  // float voltage0 = roundf((val0 * 5.0 / 1024.0) * 10.0) / 10.0;
  float voltage0 = String((val0 * 5.0 / 1024.0), 3).toFloat();
  // float voltage1 = roundf((val1 * 5.0 / 1024.0) * 10.0) / 10.0;
  float voltage1 = String((val1 * 5.0 / 1024.0), 3).toFloat();

  //pressure0 = (voltage0 - 1.0) * 2.5 ; // range: 10 bar. (10.0 / 4.0)
  //pressure1 = (voltage1 - 1.0) * 2.5 ;

  pressure0 = String((voltage0 - 1.0) * 2.5, 3).toFloat();
  pressure1 = String((voltage1 - 1.0) * 2.5, 3).toFloat();
      
  String str_pressure = String(pressure0) + ',' + String(pressure1);
  Serial.println(str_pressure);  
  Serial2.println(str_pressure);
  // String str_voltage = String(voltage0) + ',' + String(voltage1);
  // Serial.println(str_voltage); 
  
  delay(200);
}
