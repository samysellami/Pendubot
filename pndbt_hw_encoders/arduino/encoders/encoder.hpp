#include <SPI.h>

#define FILTER_SAMPLE 10

class Encoder
{
public:

  float offset = 0;

  void init(int slave_pin, float offset_)
  {
    slavePin = slave_pin;
    error_bit = 0;
    offset = offset_;
    pinMode(slavePin, OUTPUT);

    for(int i = 0; i < FILTER_SAMPLE; i++) f_buff[i] = 0;
    
  }

  long mod_(long dividend, long divisor)
  {
    long r = dividend % divisor;
    return (r < 0) ? (r + divisor) : r;
  }

  unsigned long convert(uint8_t *arr)
  {
    unsigned long res = 0;
    res = arr[0];
    res = res << 8;
    res = res | arr[1];
    res = res << 4;
    res = res | (arr[2] >> 4);
    return res;
  }

  unsigned long readValue()
  {
    digitalWrite(slavePin, LOW);
    for (int i = 0; i < 5; i++)
    {
      data[i] = SPI.transfer(0);
    }
    digitalWrite(slavePin, HIGH);
    error_bit = (data[2] & 0b00000100) >> 2;

    abs_pos = convert(data);
    if(abs_pos > cpr ) return old_abs_pos;    
    old_abs_pos = abs_pos;
    
    return abs_pos;
  }

  float GetAngleRad()
  {
    return pos_abs_rad_;
  }

  float GetVel()
  {
    return vel_abs_filter_;
  }

  float GetVelF()
  {
    return vel_abs_filter2_;
  }

  float GetDt()
  {
    return dtime;
  }

  float GetDPos()
  {
    return d_pos;
  }

  long read()
  {
    readValue();

    if (error_bit == 0)
    {
      current_time = micros();
      dtime = (float)(current_time - old_time) * 0.000001;//(1.0f / 1000000.0f);
      
      delta_enc = abs_pos - count_in_cpr_;
      delta_enc = mod_(delta_enc, cpr);
      if (delta_enc > cpr / 2)
        delta_enc -= cpr;      
 
      shadow_count_ += delta_enc;
      count_in_cpr_ += delta_enc;
      count_in_cpr_ = mod_(count_in_cpr_, cpr);      

      // Get abs angle rad
      pos_abs_rad_ = ((float)shadow_count_ * 2 * M_PI * (1.0f / (float)(cpr)) ) - offset;

      
      //--------------------------------------------------
      
      //d_pos = pos_abs_rad_;
      //float pot_raw = (pos_abs_rad_ - old_pos_abs_rad_) / dtime;
      //vel_abs_filter_ = pot_raw;
      
      //old_pos_abs_rad_ = pos_abs_rad_;
      
      f_buff[f_index] = pos_abs_rad_;
      time_Buff[f_index] = current_time;

      if (f_index >= FILTER_SAMPLE - 1)
        f_index = 0;
      else
        f_index++;

      vel_abs_filter_ = (pos_abs_rad_ - f_buff[f_index]) / ((float)(current_time - time_Buff[f_index]) * 0.000001);
      
      
/*
      float deltaf = fabs(vel_abs_filter_ - old_vel_abs_filter_);
      if(isVel == 6 && (deltaf > 7))
      {
        Serial.print("Vel: ");
        Serial.print(vel_abs_filter_,4);
        Serial.print(" f_buff: ");
        Serial.print(f_buff[f_index],4);
        Serial.print(" Pos: ");
        Serial.print(pos_abs_rad_,4);
        Serial.print(" dt: ");
        Serial.print(dtime,6);
        Serial.println();
        delay(100000);
      }
      
      if(isVel == 1) old_vel_abs_filter_ = vel_abs_filter_;
      */
      
      old_time = current_time;
    }

    return shadow_count_;
  }

private:
  int slavePin;
  uint8_t data[5];
  int error_bit;

  long count_in_cpr_ = 0;
  long delta_enc = 0;
  unsigned long cpr = 522239;
  long shadow_count_ = 0;

  unsigned long abs_pos = 0;
  unsigned long old_abs_pos = 0;

  float pos_abs_rad_ = 0;
  float old_pos_abs_rad_ = 0;

  float f_buff[FILTER_SAMPLE];
  uint8_t f_index = 0;

  float vel_abs_filter_ = 0.0f;
  float vel_abs_filter2_ = 0;

  unsigned long current_time = 0;
  unsigned long old_time = 0;
  unsigned long time_Buff[FILTER_SAMPLE];

  float dtime = 0;
  float d_pos = 0;

  //uint8_t isVel = 0;
};
