#ifndef READDATAROS_H
#define READDATAROS_H

#include "ros/ros.h"

class ReadDataRos
{
public:
  ReadDataRos();
  void addSubscribe();

protected:
  void onPlanningSubscirbe(std_msgs::String::ConstPtr);

private:
  std::shared_ptr<ros::NodeHandle> m_pNodeHandle;
  ros::Subscriber m_subPlanning;
  ros::Subscriber m_subPerception;
};

#endif // READDATAROS_H
