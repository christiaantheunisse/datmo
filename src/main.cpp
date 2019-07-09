#include "ros/ros.h"
#include "datmo.h"
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "datmo_node");

  //Create an object of class datmo 
  Datmo  datmo_object;

  ros::spin();

  return 0;
}