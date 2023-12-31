#include <Wire.h>
#include <Thread.h>
#include <PID_v1.h>
#include <TimerOne.h>


const int pinV0 = 3;
const int pinV1 = 4;
const int ledPin = LED_BUILTIN;
int ledState = LOW;

// high is closed, low is open.

//-- I2C structure
typedef struct __attribute__ ((packed)) {
  byte cmd;
  byte No;
  float value0;
  float value1;
  float value2;
  float value3;
  float value4;
  float value5;
  float value6;
} I2CTransferStruct;

I2CTransferStruct I2CData;
byte * pI2CData = NULL;
bool isI2CDataNew = true;
//-- end of I2C structure

//-- PID 
typedef struct __attribute__ ((packed)) {
  double setPoint;
  double input;
  double output;
  double kp;
  double ki;
  double kd;
  int flipCnt;
  int pin;
  PID *pPID;
} PIDStruct;
PIDStruct PIDValve0;
PIDStruct PIDValve1;

PID myPID0(&(PIDValve0.input), &(PIDValve0.output), &(PIDValve0.setPoint), PIDValve0.kp, PIDValve0.ki, PIDValve0.kd, DIRECT);
PID myPID1(&(PIDValve1.input), &(PIDValve1.output), &(PIDValve1.setPoint), PIDValve1.kp, PIDValve1.ki, PIDValve1.kd, DIRECT);

byte selfPIDStarted = 0;
//-- end of PID 

//-- Thread
Thread PIDThread = Thread();
//-- end of thread

void setup()
{
  Wire.begin(10);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);

  pinMode(pinV0, OUTPUT);
  pinMode(pinV1, OUTPUT);
  // pinMode(LED_BUILTIN, OUTPUT);

  // S070C-SBG-32
  // high is closed, low is open.
  digitalWrite(pinV0, HIGH); 
  digitalWrite(pinV1, HIGH);

  pI2CData = (byte *) &I2CData;

  PIDThread.onRun(PIDThreadLoop);
  PIDThread.setInterval(200);
  Timer1.initialize(10000); // 0.01 s
  Timer1.attachInterrupt(timerCallback);

  PIDValve0.pPID = &myPID0;
  PIDValve0.pin = pinV0;
  PIDValve1.pPID = &myPID1;
  PIDValve1.pin = pinV1;
}

void loop()
{
  // noInterrupts();
  // Serial.println("loop");
  // interrupts();
  ThreadLoop();
  delay(100);
}

void timerCallback()
{
  PIDThread.run();
}

void receiveEvent(int howmany)
{
  char buf[10] = "";
  int idx = 0;
  
  while(Wire.available() >= 1)
  {
    *(pI2CData + idx) = Wire.read();
    idx ++;
  }
  // Serial.print(I2CData.cmd);
  // Serial.println(",");
  // Serial.print(I2CData.value1);
  isI2CDataNew = true;
}

void ThreadLoop(void)
{
  // Serial.print("PIDThreadLoop \n");

  if (isI2CDataNew) {
    isI2CDataNew = false;
    switch (I2CData.cmd)
    {
    case 10: // update outputs
      PIDValve0.input = I2CData.value0;
      PIDValve0.output = I2CData.value1;
      PIDValve0.setPoint = I2CData.value2;
      PIDValve1.input = I2CData.value3;
      PIDValve1.output = I2CData.value4;
      PIDValve1.setPoint = I2CData.value5;
      selfPIDStarted = 2; // start ML PID
      break;
    case 11: // update inputs, setpoints
      PIDValve0.input = I2CData.value0;
      PIDValve0.setPoint = I2CData.value1;
      PIDValve1.input = I2CData.value2;
      PIDValve1.setPoint = I2CData.value3;
      selfPIDStarted = 1; // start self PID
      break;
    case 20: // update coefficients
      PIDValve0.kp = I2CData.value0;
      PIDValve0.ki = I2CData.value1;
      PIDValve0.kd = I2CData.value2;
      PIDValve1.kp = I2CData.value3;
      PIDValve1.ki = I2CData.value4;
      PIDValve1.kd = I2CData.value5;
      break;
    case 21: 
      break;
    case 22: 
      break;
    
    default:
      break;
    }
  }
}

void PIDThreadLoop()
{
  // noInterrupts();
  if (selfPIDStarted) {
    if (selfPIDStarted == 1) {
    // run PID algorithm to get output
      PIDCompute(&PIDValve0);
      PIDCompute(&PIDValve1);
    }
    // Serial.println("\r\n start to do outputs");
    outputs(&PIDValve0);
    //outputs(&PIDValve1);
    selfPIDStarted = 0;
  }
  // interrupts();
}

void outputs(PIDStruct *structPID)
{
  if (structPID == NULL)
    return;

  int timeCnt = 10; // keep two points
  structPID->flipCnt ++;

  float output = (structPID->output);
  if (output < 0) {
    openValve(false, structPID->pin);
  }
  else{
    int flip = output > 1 ? timeCnt : (int(output * timeCnt));
    float diff = structPID->setPoint - structPID->input;

    // strCombinePrint(structPID->setPoint, structPID->input, structPID->flipCnt, flip);

    // if ((output > 0) && (structPID->flipCnt % timeCnt < flip) && (structPID->setPoint != 0)) {
    if ((structPID->flipCnt % timeCnt < flip) && (structPID->setPoint != 0)) {
      openValve(true, structPID->pin);
    }
    else {
      openValve(false, structPID->pin);
    }
  }
}

void strCombinePrint(float input0, float input1, float input2, float input3)
{
  String str = String(input0) + ',' + String(input1) + ',' + String(input2)+ ',' + String(input3);
  Serial.println(str);  
}

void openValve(bool open, int pin)
{
  if (open) {
    digitalWrite(pin, LOW);
  }
  else {
    digitalWrite(pin, HIGH);
  }
}

void PIDCompute(PIDStruct *structPID)
{
  float diff = structPID->setPoint - structPID->input;
  if (abs(diff > 0.00)) {
    structPID->pPID->SetTunings(structPID->kp, structPID->ki, structPID->kd);
  }
  else if (abs(diff <= 0.05)) {
    //we're close to setpoint, use conservative tuning parameters
    // structPID->pPID->SetTunings(consKp, consKi, consKd);      
  }

  structPID->pPID->Compute();
}
