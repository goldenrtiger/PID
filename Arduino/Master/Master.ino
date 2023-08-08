#include <Arduino.h>

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

void setup() {
  // analogReference (EXTERNAL) ;
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  
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

void loop() {
  // put your main code here, to run repeatedly:
  averageAnalogReading(pin_P0In, pin_P1In, &val0, &val1, false);
  val0 = val0 * val0_offset;
  val1 = val1 * val1_offset;
  
  //-- receive message for calibration
  if (Serial.available() >= 1)
  {
    rStr = Serial.readString();
    rStr.trim();
    if (rStr == "Calibration")
    {      
      val0_offset = voltageCalibrate(val0);
      val1_offset = voltageCalibrate(val1);
      
      Serial.write("Calibration done!\r\n");
      strCombinePrint(val0_offset, val1_offset);
      Serial2.write("Calibration done!\r\n");
      Serial2.println(String(val0_offset) + ',' + String(val1_offset));
    }
  }
  //-- end of calibration

  float voltage0 = analog2Voltage(val0);
  float voltage1 = analog2Voltage(val1);

  pressure0 = voltage2Pressure(voltage0);
  pressure1 = voltage2Pressure(voltage1);

  strCombinePrint(voltage0, voltage1);
  strCombinePrint(pressure0, pressure1);      
  
  delay(200);
}