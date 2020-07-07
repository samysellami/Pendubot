#ifndef __IMUMODULE_HPP
#define __IMUMODULE_HPP

#include <cstdlib>
#include <fstream>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <string>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "imu_sensor/ImuData.h"

namespace pt = boost::posix_time;
//#include "UDPServer.hpp"

ros::Publisher *topic_pub_ptr;

class ImuModule
{
public:
  pt::ptime start_rec_time;
  unsigned int esp_start_rec_time = 0;

  ImuModule(boost::asio::io_service &io_service, int id) : of(), Timer1(io_service, boost::posix_time::millisec(1000))
  {
    id_ = id;
    Timer1.async_wait(boost::bind(&ImuModule::Timer1_Hendle, this, boost::asio::placeholders::error, &Timer1));
  }

  struct DataIMU
  {
    float AccelX;
    float AccelY;
    float AccelZ;

    float GyroX;
    float GyroY;
    float GyroZ;

    float MagX;
    float MagY;
    float MagZ;

    // float Temperature;

    float q0;
    float q1;
    float q2;
    float q3;
  };

  struct ESP_IMU
  {
    DataIMU imu;

    bool Synch;
    unsigned int Time;
  };
  //---------------------------------------------------------------
  ~ImuModule()
  {
  }
  //---------------------------------------------------------------
  void StartRec(std::string Patch)
  {
    std::string name = "logs/" + Patch + "/" + std::to_string(id_) + "-" + name_ + ".txt";
    of.open(name);

    start_rec_time = pt::microsec_clock::local_time();
    isStart = true;
  }
  //---------------------------------------------------------------
  void StopRec()
  {
    isStart = false;
    esp_start_rec_time = 0;
    of.close();
  }
  //---------------------------------------------------------------
  void Timer1_Hendle(const boost::system::error_code & /*e*/, boost::asio::deadline_timer *t)
  {
    t->expires_at(t->expires_at() + boost::posix_time::millisec(1000));
    t->async_wait(boost::bind(&ImuModule::Timer1_Hendle, this, boost::asio::placeholders::error, t));

    FPS = NumberReceivPack;
    NumberReceivPack = 0;
  }
  //---------------------------------------------------------------
  float GetRoll(DataIMU dataIMU)
  {
    float Roll = -atan2(2.0f * (dataIMU.q0 * dataIMU.q2 - dataIMU.q1 * dataIMU.q3),
                        1.0f - 2.0f * (dataIMU.q2 * dataIMU.q2 + dataIMU.q1 * dataIMU.q1)) *
                 57.2;
    if (Roll < 0)
      Roll += 360.0;
    return Roll;
  }
  //---------------------------------------------------------------
  float GetPitch(DataIMU dataIMU)
  {
    // calculate pitch angles
    float Pitch = atan2(2 * dataIMU.q2 * dataIMU.q3 - 2 * dataIMU.q0 * dataIMU.q1,
                        2 * dataIMU.q0 * dataIMU.q0 + 2 * dataIMU.q3 * dataIMU.q3 - 1) *
                  57.2;
    if (Pitch < 0)
      Pitch += 360.0;
    return Pitch;
  }
  //---------------------------------------------------------------
  float GetYaw(DataIMU dataIMU)
  {
    // calculate yaw angles
    return atan2(2 * dataIMU.q1 * dataIMU.q2 - 2 * dataIMU.q0 * dataIMU.q3,
                 2 * dataIMU.q0 * dataIMU.q0 + 2 * dataIMU.q1 * dataIMU.q1 - 1) *
           57.2;
  }
  //---------------------------------------------------------------
  void Update(ESP_IMU esp_imu)
  {
    ConnectionStatus = true;
    NumberReceivPack++;

    pt::ptime current_date_microseconds = pt::microsec_clock::local_time();

    // long milliseconds = current_date_microseconds.time_of_day().total_milliseconds();
    // pt::time_duration current_time_milliseconds = pt::milliseconds(milliseconds);
    // pt::ptime current_date_milliseconds(current_date_microseconds.date(),
    // current_time_milliseconds);
    // std::cout << current_date_microseconds << std::endl;
    //<< " Milliseconds: " << current_date_milliseconds << std::endl;

    /*
    if (esp_imu.Synch == 1 && isStart == false)
    {
        boost::posix_time::ptime utcCur = boost::posix_time::second_clock::local_time();
        //std::cout << utcCur << "\n";
        std::string Patch = to_simple_string(utcCur);
        boost::filesystem::create_directory("logs/" + Patch);

        StartRec(Patch);
    }

    if (esp_imu.Synch == 0 && isStart == true)
    {
        StopRec();
    }
    */

    if (isStart)
    {
      if (esp_start_rec_time == 0)
        esp_start_rec_time = esp_imu.Time;

      data.Time = (esp_imu.Time - esp_start_rec_time) / 1000.0f;
      data.AccelX = esp_imu.imu.AccelX;
      data.AccelY = esp_imu.imu.AccelY;
      data.AccelZ = esp_imu.imu.AccelZ;
      data.GyroX = esp_imu.imu.GyroX;
      data.GyroY = esp_imu.imu.GyroY;
      data.GyroZ = esp_imu.imu.GyroZ;
      data.MagX = esp_imu.imu.MagX;
      data.MagY = esp_imu.imu.MagY;
      data.MagZ = esp_imu.imu.MagZ;
      data.q0 = esp_imu.imu.q0;
      data.q1 = esp_imu.imu.q1;
      data.q2 = esp_imu.imu.q2;
      data.q3 = esp_imu.imu.q3;


      /*
      // current_date_microseconds.time_of_day().total_microseconds()
      of << (current_date_microseconds - start_rec_time).total_milliseconds() / 1000.0f << " "
         << (esp_imu.Time - esp_start_rec_time) / 1000.0f << " " << esp_imu.imu.AccelX << " " << esp_imu.imu.AccelY
         << " " << esp_imu.imu.AccelZ << " " << esp_imu.imu.GyroX << " " << esp_imu.imu.GyroY << " "
         << esp_imu.imu.GyroZ << " " << esp_imu.imu.MagX << " " << esp_imu.imu.MagY << " " << esp_imu.imu.MagZ << "
      "
         << esp_imu.imu.q0 << " " << esp_imu.imu.q1 << " " << esp_imu.imu.q2 << " " << esp_imu.imu.q3 << " "
         << GetYaw(esp_imu.imu) << " " << GetPitch(esp_imu.imu) << " " << GetRoll(esp_imu.imu) << " ";  // short
      macro of << "\n";
        */

      topic_pub_ptr->publish(data);

      // std::cout << GetPitch(esp_imu.imu) << " " << GetRoll(esp_imu.imu) << std::endl;
    }
  }

  //---------------------------------------------------------------
  boost::asio::ip::address GetIpAddress()
  {
    return ip;
  }
  //---------------------------------------------------------------
  void SetIpAddress(boost::asio::ip::address ip_)
  {
    ip = ip_;
  }
  //---------------------------------------------------------------
  void SetName(std::string name)
  {
    name_ = name;
  }
  //----------------------------------------------------------------
  std::string GetName()
  {
    return name_;
  }
  //-----------------------------------------------------------------
  int GetFPS()
  {
    return FPS;
  }

private:
  boost::asio::ip::address ip;
  std::string name_;

  boost::asio::deadline_timer Timer1;
  // boost::asio::deadline_timer Timer_TimeOut;

  int FPS;
  unsigned int NumberReceivPack;

  bool ConnectionStatus;

  std::ofstream of;

  boost::asio::io_service io_service2;

  bool isStart = false;
  int id_ = 0;

  imu_sensor::ImuData data;
};

#endif