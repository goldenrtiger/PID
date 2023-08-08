#include <Wire.h>

void setup()
{
  Wire.begin(10);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

void loop()
{
  delay(100);
}

void receiveEvent(int howmany)
{
  char buf[10] = "";
  int idx = 0;
  
  while(Wire.available() >= 1)
  {
    buf[idx] = Wire.read();
    idx ++;
  }
  Serial.print(buf);

}
