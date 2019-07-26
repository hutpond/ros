#include "std_msgs/String.h"
#include "ReadDataRos.h"

ReadDataRos::ReadDataRos()
{
  m_pNodeHandle.reset(new ros::NodeHandle);
}

void ReadDataRos::addSubscribe()
{
  m_subPlanning = m_pNodeHandle->subscribe<std_msgs::String::ConstPtr>(
        "/scan",
        1,
        &ReadDataRos::onPlanningSubscirbe,
        this
        );
}

void ReadDataRos::onPlanningSubscirbe(std_msgs::String::ConstPtr)
{

}
