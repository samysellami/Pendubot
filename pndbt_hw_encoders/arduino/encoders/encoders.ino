#include "encoder.hpp"

Encoder encoder1;
Encoder encoder2;

long result1 = 0;
long result2 = 0;

unsigned long current_time = 0;
unsigned long old_time = 0;

SPISettings settings(3000000, MSBFIRST, SPI_MODE0);

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(settings);
  encoder1.init(10,-2.6758);
  encoder2.init(9,1.61844);
  delay(100);

  for(int i = 0; i < 10; i++)
  {
    result1 = encoder1.readValue();
    result2 = encoder2.readValue();
  }
}

unsigned int hz_count = 0;
unsigned int hz = 0;

unsigned long send_time = 0;
unsigned long send_time2 = 0;

void loop()
{
  current_time = micros();
  //hz_count++;
  
  //float dtime = (float)(current_time - old_time) * 0.000001;//(1.0f / 1000000.0f);
/*
  if (send_time2 == 0)
    send_time2 = current_time;
  if (current_time - send_time2 >= 1000000)
  {
    send_time2 = 0;
    hz = hz_count;
    hz_count = 0;
  }
   */

  result1 = encoder1.read();
  result2 = encoder2.read();
  

  if (send_time == 0)
    send_time = current_time;
  if (current_time - send_time >= 2000)
  {
    send_time = 0;

    Serial.print(encoder1.GetAngleRad(),3);
    Serial.print(" ");
    Serial.print(encoder1.GetVel(),3);
    Serial.print(" ");
    Serial.print(encoder2.GetAngleRad(),3);
    Serial.print(" ");
    Serial.print(encoder2.GetVel(),3);
    //Serial.print(" ");
    
    //Serial.print(int(1 / dtime));
    //Serial.print(" ");
    //Serial.print(hz);
    Serial.println();
  }

  //old_time = current_time;
}
