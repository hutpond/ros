#ifndef CHOSTAPI4HMI_H
#define CHOSTAPI4HMI_H

#include "ads_msgs/ads_hmi_command.h"

#include "ads_msgs/ads_module_report.h"
#include "ads_msgs/ads_control_WheelPositionReport.h"
#include "ads_msgs/ads_control_brakereport.h"
#include "ads_msgs/ads_control_fuel_level_report.h"
#include "ads_msgs/ads_control_steering_report.h"
#include "ads_msgs/ads_control_throttle_report.h"
#include "ads_msgs/ads_device_report.h"

#include "dbAdsApi4HMI.h"

class CHostApi4HMI : public dbAds::IHostApi4HMI
{
public:
  CHostApi4HMI();

  virtual void getNodeHandle(std::vector<ros::NodeHandle*> &) override;
  void OnMsg_ads_hmi_command(const ads_msgs::ads_hmi_command &);
  void run();

  dbAds::IApi4HMI* m_lpApi = nullptr;

protected:
  ros::NodeHandle* m_lpnode = nullptr;
  ros::NodeHandle *m_nodeHandle = nullptr;

  std::map<std::string, ros::Subscriber> m_Subs;
  std::map<std::string, ros::Publisher> m_Pubs;
};

#endif // CHOSTAPI4HMI_H
