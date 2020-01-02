#include <thread>
#include "CHostApi4HMI.h"

CHostApi4HMI::CHostApi4HMI()
{
  m_lpnode = new ros::NodeHandle("sweeper_hmi_node_");

  m_lpApi = dbAds::getApi4HMI();
  m_lpApi->StartWithHost(this);

  std::vector<std::string> cared_property = { dbAds::IApi4HMI::Item_manual_brake
                                              , dbAds::IApi4HMI::Item_width_light
                                              , dbAds::IApi4HMI::Item_low_beam_light
                                              , dbAds::IApi4HMI::Item_high_beam_light
                                              , dbAds::IApi4HMI::Item_emergency_light
                                              , dbAds::IApi4HMI::Item_fault_list
                                              , dbAds::IApi4HMI::Item_brush_status
                                              , dbAds::IApi4HMI::Item_spout_water
                                              , dbAds::IApi4HMI::Item_car_state
                                              , dbAds::IApi4HMI::Item_module_state_changed
                                              , dbAds::IApi4HMI::Item_water_level
                                              , dbAds::IApi4HMI::Item_gear
                                             };
  m_lpApi->InstallEventCallback([this](dbAds::IApi4HMI::CEventReport& report)
  {
      if(dbAds::IApi4HMI::Item_module_state_changed == report.m_ItemName)
      {
          std::cout << "report[\"" << report.m_ItemName << "\"]=\""
              << boost::any_cast<std::string>(report.m_value)
              << "\""
              << std::endl;
#if 0
          std::vector<dbAds::IApi4HMI::CFaultInfo> faults;
          m_lpApi->GetFaultList(faults);
          for (auto it = faults.begin(); it != faults.end(); it++)
          {
              std::cout << *it << std::endl;
          }
#endif
      }
      return 1;
  }, cared_property);

#if 0
  std::string topic = "/ads_ad_command";
  m_Subs[topic] = m_lpnode->subscribe(topic, 10, &CHostApi4HMI::OnMsg_ads_ad_command, this);

  topic = "/ads_site_data_path";
  m_Subs[topic] = m_lpnode->subscribe(topic, 10, &CHostApi4HMI::OnMsg_ads_site_data_path, this);

  topic = "/ads_module_report";
  m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_module_report>(topic, 2);
#endif
}

void CHostApi4HMI::OnMsg_ads_ad_command(const ads_msgs::ads_ad_command& msg)
{
    std::cout << __FUNCTION__ << "(" << (unsigned int)msg.action << ")" << std::endl;
}

void CHostApi4HMI::OnMsg_ads_site_data_path(const ads_msgs::ads_site_data_path& msg)
{
    std::cout << __FUNCTION__ << "(\"" << msg.data << "\")" << std::endl;
}


void CHostApi4HMI::getNodeHandle(std::vector<ros::NodeHandle*> &nhs)
{
  nhs.push_back(m_lpnode);
}

void CHostApi4HMI::run()
{
  std::thread th_rosloop = std::thread([]()
  {
    pthread_setname_np(pthread_self(), "SpinOnce");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  });
  th_rosloop.detach();
  return;

  while (1)
  {
#if 0
    ads_msgs::ads_module_report msg;
    msg.deviceID = 0;
    msg.moduleID = ads_msgs::ads_module_report::moduleID_Planning;
    msg.moduleStatus = ads_msgs::ads_module_report::moduleStatus_Critical;
    msg.moduleSubtype = 1;

    std::string topic = "/ads_module_report";
    m_Pubs[topic].publish(msg);
#endif

    auto check = m_lpApi->GetSelfCheck(dbAds::ISelfCheck::e_type::System);

    check->Start();
    auto progress_value = check->GetProgress();
    while(check->GetResult() == dbAds::ISelfCheck::Processing)
    {
      if(progress_value != check->GetProgress())
      {
        progress_value = check->GetProgress();
        std::cout << check->GetProgress() << "%" << std::endl;
      }
    }
    check->Stop();
    std::cout << check->GetProgress() << "%" << std::endl;
    std::cout << "Result:" << check->GetResult() << std::endl;

    auto values = m_lpApi->GetProperty({ dbAds::IApi4HMI::Item_manual_brake
                                         , dbAds::IApi4HMI::Item_width_light
                                         , dbAds::IApi4HMI::Item_low_beam_light
                                         , dbAds::IApi4HMI::Item_high_beam_light
                                         , dbAds::IApi4HMI::Item_emergency_light
                                         , dbAds::IApi4HMI::Item_fault_list
                                         , dbAds::IApi4HMI::Item_brush_status
                                         , dbAds::IApi4HMI::Item_spout_water
                                       }
                                       );

    for (auto it = values.begin(); it != values.end(); it++)
    {
        if (it->second.type() == typeid (int))
        {
            auto value = boost::any_cast<int>(it->second);
            std::cout << it->first << "=" << value << std::endl;
        }
        else if (it->second.type() == typeid (float))
        {
            auto value = boost::any_cast<float>(it->second);
            std::cout << it->first << "=" << value << std::endl;
        }
        else if (it->second.type() == typeid (double))
        {
            auto value = boost::any_cast<double>(it->second);
            std::cout << it->first << "=" << value << std::endl;
        }
        else if (it->second.type() == typeid (std::string))
        {
            auto value = boost::any_cast<std::string>(it->second);
            std::cout << it->first << "=\"" << value << "\"" << std::endl;
        }
        else if (it->second.type() == typeid (ads_msgs::ads_ad_report::_carState_type))
        {
            auto value = boost::any_cast<ads_msgs::ads_ad_report::_carState_type>(it->second);
            std::cout << it->first << "=" << (int)value << std::endl;
        }
        else
        {
            std::cout << it->first << " with unsuportd type:" << it->second.type().name() << std::endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}
