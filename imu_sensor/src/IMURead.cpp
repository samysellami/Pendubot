#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "ConsoleTerminal.h"
#include "UDPServer.hpp"

using boost::asio::ip::tcp;

boost::asio::io_service io_service;

boost::thread *terminal_ptr;

//------------------------------------------------------
void my_function(int sig)
{  // can be called asynchronously
  // flag = 1;  // set flag
  std::cout << "quit command\n";

  io_service.stop();
  terminal_ptr->interrupt();

  signal(SIGINT, my_function);
}

//----------------------------------------------------------------------------------------------------------------------------
void ThreadTerminal()
{
  setlocale(LC_ALL, "Russian");

  // Register signals
  signal(SIGINT, my_function);

  InitCommand();

  for (std::string line; std::getline(std::cin, line);)
  {
    std::istringstream iss(line);

    std::string commandName;
    iss >> commandName;

    auto const it = commandMap.find(commandName);
    if (it != std::end(commandMap))
    {
      command::args_type const commandArgs(std::istream_iterator<std::string>{ iss },
                                           std::istream_iterator<std::string>{});
      it->second->exec(commandArgs);
    }
    else
    {
      std::cerr << "'" << commandName << "' - Unknown command" << std::endl;
    }
  }

  std::cout << "quit - ThreadTerminal...\n";
}
//----------------------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_read");
  ros::NodeHandle n;
  ros::Publisher topic_pub = n.advertise<imu_sensor::ImuData>("/pndbt/imu_sensor", 1000);
  topic_pub_ptr = &topic_pub;

  boost::thread TT(ThreadTerminal);
  terminal_ptr = &TT;

  if (!boost::filesystem::exists("logs"))
    boost::filesystem::create_directory("logs");

  try
  {
    UDPServer server(io_service, 4442);
    udp_server = &server;
    io_service.run();
  }
  catch (std::exception &e)
  {
    std::cerr << "Exception:  " << e.what() << "\n";
  }
  /*
    int count = 0;
    while (ros::ok())
    {
      std_msgs::String msg;

      imu_sensor::ImuData data;

      data.data1 = count;

      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());

      chatter_pub.publish(data);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }
    */

  return 0;
}
