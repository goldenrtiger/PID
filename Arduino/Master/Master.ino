#include <Arduino.h>
#include <Thread.h>
#include <Wire.h>
#include <TimerOne.h>

int pin_P0In = A0;
int pin_P1In = A1;

float val0, val1;
int started_flag = 0;
String rStr;
float val0_offset = 1.0;
float val1_offset = 1.0;

float P0Value = 0.0;
float P1Value = 0.0;

float pressure0 = 0.0;
float pressure1 = 0.0;

float refVoltage = 5.0;

//-- Thread
Thread commThread = Thread();
//-- end of thread

//-- Wire
int wireAddres0 = 10;
//-- end of Wire

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
//-- end of I2C structure

void setup() {
  // analogReference (EXTERNAL) ;
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial2.begin(115200);

  commThread.onRun(commThreadLoop);
  commThread.setInterval(200);
  Timer1.initialize(200000); // 0.2 s
  Timer1.attachInterrupt(timerCallback);

  Wire.begin();  
  pI2CData = (byte *)&I2CData;
}

void loop() {
  // put your main code here, to run repeatedly:
  MLSerial();
  wireWrite();   
  
  delay(200);
}

void commThreadLoop(void)
{
  // Serial.print("commThreadLoop \n");
  analog2Pressure();
  pressureSending(); 
}

void timerCallback()
{
  commThread.run();
}

void wireWrite()
{
  if (I2CData.cmd ) {
    byte cmd = I2CData.cmd;
    Wire.beginTransmission(wireAddres0);    
    if (I2CData.cmd == 10) { // update outputs
      I2CData.value0 = pressure0;
      I2CData.value3 = pressure1;
      for(int i = 0;i < sizeof(I2CData);i++)
      {
        Wire.write(pI2CData[i]);
      }
      I2CData.cmd = 0;
    }
    else if (I2CData.cmd == 11) { // update inputs, setpoints
      I2CData.value0 = pressure0; // valve0 input
      I2CData.value2 = pressure1; // valve1 input
      I2CData.cmd = 0;
    }
    else if (I2CData.cmd == 20 || I2CData.cmd == 21 || I2CData.cmd == 22) { // update coefficients
      I2CData.cmd = 0;
    }

    Wire.write(cmd);
    for(int i = 1;i < sizeof(I2CData);i++)
    {
      Wire.write(pI2CData[i]);
    }
    Wire.endTransmission();
  }
}

void MLSerial()
{
  //-- receive message for calibration, ,,,
  if (Serial.available() >= 1) {
    rStr = Serial.readString();
    rStr.trim();
    if (rStr.startsWith("Calibration")) {      
      val0_offset = voltageCalibrate(val0);
      val1_offset = voltageCalibrate(val1);
      
      Serial.write("Calibration done!\r\n");
      // strCombinePrint(val0_offset, val1_offset);
      Serial2.write("Calibration done!\r\n");
      Serial2.println(String(val0_offset) + ',' + String(val1_offset));
    }
    else if (rStr.startsWith("PIDML")) { // "PIDML:0.1:0.2:0.3:0.4 \n"
      byte idxBuf[10];
      splitDelimiter(rStr, ':', idxBuf);
      I2CData.value1 = (rStr.substring(idxBuf[1], idxBuf[2] - 1)).toFloat(); // output
      I2CData.value2 = (rStr.substring(idxBuf[2], idxBuf[3] - 1)).toFloat(); // setPoint
      I2CData.value4 = (rStr.substring(idxBuf[3], idxBuf[4] - 1)).toFloat(); // output
      I2CData.value5 = (rStr.substring(idxBuf[4], idxBuf[5] - 1)).toFloat(); // setPoint
      I2CData.cmd = 10;
    }
    else if (rStr.startsWith("PIDSelf")) { // "PIDSelf:0.1:0.2 \n"
      I2CData.cmd = 11;
      I2CData.value1 = getValue(rStr, ':', 1).toFloat(); //setPoint
      I2CData.value3 = getValue(rStr, ':', 2).toFloat(); //setPoint
    }
    else if (rStr.startsWith("PIDCoeff")) { // "PIDCoeff:0.1:0.2:0.3:0.4:0.5:0.6 \n"
      I2CData.cmd = 20;
      I2CData.value0 = getValue(rStr, ':', 1).toFloat(); // kp
      I2CData.value1 = getValue(rStr, ':', 2).toFloat(); // ki
      I2CData.value2 = getValue(rStr, ':', 3).toFloat(); // kd
      I2CData.value3 = getValue(rStr, ':', 4).toFloat(); // kp
      I2CData.value4 = getValue(rStr, ':', 5).toFloat(); // ki
      I2CData.value5 = getValue(rStr, ':', 6).toFloat(); // kd

    }
  }
  //-- end of calibration
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

void splitDelimiter(String data, char delimiter, byte* pIdxBuf)
{
  if(pIdxBuf == NULL)
    return; 

  int maxIndex = data.length() - 1;
  int j = 0;

  *(pIdxBuf + j) = 0;
  for(int i = 0;i <= maxIndex;i++) {
    if (data.charAt(i) == delimiter) {
      j++;
      *(pIdxBuf + j) = i + 1;
    }
  }
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

void analog2Pressure()
{
  averageAnalogReading(pin_P0In, pin_P1In, &val0, &val1, false);
  val0 = val0 * val0_offset;
  val1 = val1 * val1_offset;
  
  float voltage0 = analog2Voltage(val0);
  float voltage1 = analog2Voltage(val1);

  pressure0 = voltage2Pressure(voltage0);
  pressure1 = voltage2Pressure(voltage1);
}

void pressureSending()
{
  //strCombinePrint(voltage0, voltage1);
  strCombinePrint(pressure0, pressure1);   
}

float voltageCalibrate(float input)
{
  //-- initial voltage is 1.0 volt
  float output = 0.0;

  if (1) {
    output = String(1024.0 / refVoltage / input, 3).toFloat();
  }
  else {

  }

  return output;
}

float analog2Voltage(float input)
{
  float output = 0.0;

  if (1) {
    output = String((input * 5.0 / 1024.0), 3).toFloat();
  }
  else {
    output = 0.0;
  }

  return output;
}

float voltage2Pressure(float input)
{
  float output = 0.0;

  if (1) {
    output = String((input - 1.0) * 2.5, 3).toFloat(); // 2.5 = 10 bar / 4v
  }
  else {
    output = 0.0;
  }

  return output;
}

void strCombinePrint(float input0, float input1)
{
  String str = String(input0) + ',' + String(input1);
  Serial.println(str);
}
