#ifndef CHOSTAPI4HMI_H
#define CHOSTAPI4HMI_H

#include "ads_msgs/ads_ad_command.h"
#include "ads_msgs/ads_site_data_path.h"
#include "ads_msgs/ads_ad_report.h"
#include "ads_msgs/ads_module_report.h"
#include "dbAdsApi4HMI.h"

class CHostApi4HMI : public dbAds::IHostApi4HMI
{
public:
  CHostApi4HMI();

  virtual void getNodeHandle(std::vector<ros::NodeHandle*> &) override;

  void OnMsg_ads_ad_command(const ads_msgs::ads_ad_command& msg);
  void OnMsg_ads_site_data_path(const ads_msgs::ads_site_data_path& msg);

  void run();

  dbAds::IApi4HMI* m_lpApi = nullptr;

protected:
  ros::NodeHandle* m_lpnode = nullptr;
  ros::NodeHandle *m_nodeHandle = nullptr;

  std::map<std::string, ros::Subscriber> m_Subs;
  std::map<std::string, ros::Publisher> m_Pubs;
};

#endif // CHOSTAPI4HMI_H
