#include "dbAdsApi4HMI.h"

#include <iostream>
#include <chrono>
#include <thread>

#include "std_msgs/String.h"
#include "ads_msgs/ads_hmi_command.h"

#include "ads_msgs/ads_module_report.h"
#include "ads_msgs/ads_control_WheelPositionReport.h"
#include "ads_msgs/ads_control_brakereport.h"
#include "ads_msgs/ads_control_fuel_level_report.h"
#include "ads_msgs/ads_control_steering_report.h"
#include "ads_msgs/ads_control_throttle_report.h"
#include "ads_msgs/ads_device_report.h"

#include "xls.h"
using namespace xls;
using namespace dbAds;

namespace{

class CSelfCheckVehicle : public ISelfCheck
{
private:
  std::thread m_th;
  int m_progress = 0;
  ISelfCheck::e_result m_result = ISelfCheck::Unknown;
  int m_bStop = false;

public:
  virtual bool Start() override
  {
    m_result = ISelfCheck::Processing;
    m_bStop = false;

    m_th = std::thread([this](){
      for(m_progress = 0; m_progress < 100; m_progress += 1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        if(m_bStop) break;
      }

      if(m_bStop)
      {
        m_result = ISelfCheck::Unknown;
      }
      else
      {
        m_progress = 100;
        m_result = ISelfCheck::Pass;
      }
    });
    return true;
  }

  virtual bool Stop() override
  {
    m_bStop = true;
    m_th.join();
    return true;
  }

  virtual int GetProgress() override
  {
    return m_progress;
  }

  virtual e_result GetResult() override
  {
    return m_result;
  }
};

class CSelfCheckSystem : public ISelfCheck
{
private:
  std::thread m_th;
  int m_progress = 0;
  ISelfCheck::e_result m_result = ISelfCheck::Unknown;
  int m_bStop = false;

public:
  virtual bool Start() override
  {
    m_result = ISelfCheck::Processing;
    m_bStop = false;

    m_th = std::thread([this](){
      for(m_progress = 0; m_progress < 100; m_progress += 1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        if(m_bStop) break;
      }

      if(m_bStop)
      {
        m_result = ISelfCheck::Unknown;
      }
      else
      {
        m_progress = 100;
        m_result = ISelfCheck::Pass;
      }
    });
    return true;
  }

  virtual bool Stop() override
  {
    m_bStop = true;
    m_th.join();
    return true;
  }

  virtual int GetProgress() override
  {
    return m_progress;
  }

  virtual e_result GetResult() override
  {
    return m_result;
  }
};

class CSelfCheckAlgorithm : public ISelfCheck
{
private:
  std::thread m_th;
  int m_progress = 0;
  ISelfCheck::e_result m_result = ISelfCheck::Unknown;
  int m_bStop = false;

public:
  virtual bool Start() override
  {
    m_result = ISelfCheck::Processing;
    m_bStop = false;

    m_th = std::thread([this](){
      for(m_progress = 0; m_progress < 100; m_progress += 1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        if(m_bStop) break;
      }

      if(m_bStop)
      {
        m_result = ISelfCheck::Unknown;
      }
      else
      {
        m_progress = 100;
        m_result = ISelfCheck::Pass;
      }
    });
    return true;
  }

  virtual bool Stop() override
  {
    m_bStop = true;
    m_th.join();
    return true;
  }

  virtual int GetProgress() override
  {
    return m_progress;
  }

  virtual e_result GetResult() override
  {
    return m_result;
  }
};

class CSelfCheckSensor : public ISelfCheck
{
private:
  std::thread m_th;
  int m_progress = 0;
  ISelfCheck::e_result m_result = ISelfCheck::Unknown;
  int m_bStop = false;

public:
  virtual bool Start() override
  {
    m_result = ISelfCheck::Processing;
    m_bStop = false;

    m_th = std::thread([this](){
      for(m_progress = 0; m_progress < 100; m_progress += 1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        if(m_bStop) break;
      }

      if(m_bStop)
      {
        m_result = ISelfCheck::Unknown;
      }
      else
      {
        m_progress = 100;
        static bool flag = false;
        m_result = flag ? ISelfCheck::Pass : ISelfCheck::Failure;
        flag = !flag;
      }
    });
    return true;
  }

  virtual bool Stop() override
  {
    m_bStop = true;
    m_th.join();
    return true;
  }

  virtual int GetProgress() override
  {
    return m_progress;
  }

  virtual e_result GetResult() override
  {
    return m_result;
  }
};

class CApi4HMI : public dbAds::IApi4HMI
{
public:
  int m_bDebug = 0;

private:
  bool m_bRunning = false;

  boost::signals2::signal<int(CEventReport&)> m_eventSignal;
  boost::signals2::signal<void(const sensor_msgs::PointCloud2&)> m_CloudPointSignal;

  std::vector<ros::NodeHandle*> s_nodeHandles;

  std::map<std::string, ros::Subscriber> m_Subs;
  std::map<std::string, ros::Publisher> m_Pubs;

  std::map<std::string, ads_msgs::ads_module_report> m_module_reprots;

  IHostApi4HMI* m_hostApi = nullptr;
  xlsWorkBook* m_pWorkBook = nullptr;
  xlsWorkSheet* m_pWorkSheet = nullptr;

  std::string m_fault_code_file;

  ads_msgs::ads_control_WheelPositionReport m_msg_ads_control_WheelPositionReport;
  ads_msgs::ads_control_brakereport m_msg_ads_control_brakereport;
  ads_msgs::ads_control_fuel_level_report m_msg_ads_control_fuel_level_report;
  ads_msgs::ads_control_steering_report m_msg_ads_control_steering_report;
  ads_msgs::ads_control_throttle_report m_msg_ads_control_throttle_report;

private:
  void OnMsg_ads_module_report(const ads_msgs::ads_module_report& msg)
  {
    std::string szKey;
    szKey = std::to_string(msg.moduleID)
        + "-" + std::to_string(msg.moduleStatus)
        + "-" + std::to_string(msg.moduleSubtype);

    if (msg.deviceID > 0)
    {
      szKey = szKey + "-" + std::to_string(msg.deviceID);
    }

    switch (msg.moduleStatus)
    {
      case ads_msgs::ads_module_report::moduleStatus_Info_Online:
      default:
        return;

      case ads_msgs::ads_module_report::moduleStatus_OK:
        {
          for (auto it = m_module_reprots.begin(); it != m_module_reprots.end(); it++)
          {
            if (msg.moduleID == it->second.moduleID)
            {
              if (msg.moduleSubtype == 0)
              {
                m_module_reprots.erase(it);
              }
              else if (msg.moduleSubtype == it->second.moduleSubtype)
              {
                if (msg.deviceID == 0)
                {
                  m_module_reprots.erase(it);
                }
                else if (msg.deviceID == it->second.deviceID)
                {
                  m_module_reprots.erase(it);
                }
              }
            }
          }
        }
        break;

      case ads_msgs::ads_module_report::moduleStatus_Warning:
      case ads_msgs::ads_module_report::moduleStatus_Critical:
        {
          /*
                    std::string szKey1;
                    szKey1 = std::to_string(msg.moduleID) + "-0-0";
                    m_module_reprots.erase(szKey1);
                */
          if (msg.moduleStatus == ads_msgs::ads_module_report::moduleStatus_Critical)
          {
            ads_msgs::ads_hmi_command msg_cmd;
            msg_cmd.wRunState = 0;
            m_Pubs["/ads_hmi_command"].publish(msg_cmd);
          }
          m_module_reprots[szKey] = msg;
        }
        break;
    }

    if(m_bDebug)
    {
      std::cout << "m_module_reprots" << std::endl;
      for(auto it = m_module_reprots.begin(); it!=m_module_reprots.end();it++)
      {
        std::cout << "\t" << it->first << std::endl;
      }
    }

    CEventReport report;
    report.szCode = szKey;
    m_eventSignal(report);
  }

  void OnMsg_ads_control_WheelPositionReport(const ads_msgs::ads_control_WheelPositionReport& msg)
  {
    m_msg_ads_control_WheelPositionReport = msg;
  }

  void OnMsg_ads_control_brakereport(const ads_msgs::ads_control_brakereport& msg)
  {
    m_msg_ads_control_brakereport = msg;
  }


  void OnMsg_ads_control_fuel_level_report(const ads_msgs::ads_control_fuel_level_report& msg)
  {
    m_msg_ads_control_fuel_level_report = msg;
  }


  void OnMsg_ads_control_steering_report(const ads_msgs::ads_control_steering_report& msg)
  {
    m_msg_ads_control_steering_report = msg;
  }

  void OnMsg_ads_control_throttle_report(const ads_msgs::ads_control_throttle_report& msg)
  {
    m_msg_ads_control_throttle_report = msg;
  }

  void OnMsg_ads_device_report(const ads_msgs::ads_device_report& msg)
  {
    (void)msg;
  }

  xlsRow* getFaultRow(std::string szKey)
  {
    if (!m_pWorkBook)
    {
      m_pWorkBook = xls_open(m_fault_code_file.c_str(), "UTF-8");
      if (nullptr == m_pWorkBook)
      {
        std::cerr << "file is not excel" << std::endl;
        return nullptr;
      }

      m_pWorkSheet = xls_getWorkSheet(m_pWorkBook, 2);
      if (nullptr == m_pWorkSheet)
      {
        std::cerr << "WorkSheet is not excel" << std::endl;
        return nullptr;
      }

      xls_parseWorkSheet(m_pWorkSheet);
    }

    for (int r = 1; r <= m_pWorkSheet->rows.lastrow; r++)
    {
      xlsRow* row = &m_pWorkSheet->rows.row[r];
      BYTE* pCurCellInfo = (BYTE *)(row->cells.cell[1].str);
      if (!pCurCellInfo)
      {
        break;
      }

      if (szKey.compare(reinterpret_cast<char*>(pCurCellInfo)) == 0)
      {
        return row;
      }
    }
    std::cout << "Did not find information for \"" << szKey << "\"" << std::endl;
    return nullptr;
  }

  bool Init(void)
  {
    if(m_bRunning) return false;

    m_bRunning = true;

    m_hostApi->getNodeHandle(this->s_nodeHandles);

    std::string topic = "/ads_hmi_command";
    m_Pubs[topic] = s_nodeHandles[0]->advertise<ads_msgs::ads_hmi_command>(topic, 2);

    ros::message_traits::definition<ads_msgs::ads_hmi_command>();

    topic = "/ads_module_report";
    m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_module_report, this);

    topic = "/ads_control_WheelPositionRepor";
    m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_WheelPositionReport, this);

    topic = "/ads_control_brakereport";
    m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_brakereport, this);

    topic = "/ads_control_fuel_level_report";
    m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_fuel_level_report, this);

    topic = "/ads_control_steering_report";
    m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_steering_report, this);

    topic = "/ads_control_throttle_report";
    m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_throttle_report, this);

    topic = "/ads_device_report";
    m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_device_report, this);


    if (1)
    {
      //m_pWorkBook = xls_open(m_fault_code_file.c_str(), "UTF-8");
      m_pWorkBook = xls_open("./fault_code.xls", "UTF-8");
      if (nullptr == m_pWorkBook)
      {
        std::cerr << "file is not excel" << std::endl;
        return false;
      }

      m_pWorkSheet = xls_getWorkSheet(m_pWorkBook, 2);
      xls_parseWorkSheet(m_pWorkSheet);

      for (int r = 0; r <= m_pWorkSheet->rows.lastrow; r++)
      {
        xlsRow* row = &m_pWorkSheet->rows.row[r];
        for (int c = 0; c < m_pWorkSheet->rows.lastcol; c++)
        {
          BYTE* pCurCellInfo = (BYTE *)(row->cells.cell[c].str);
          if (nullptr != pCurCellInfo)
          {
            std::cout << pCurCellInfo;
            //getchar();
          }
        }
        std::cout << std::endl;
      }
    }

    return true;
  }

  bool DeInit(void)
  {
    if(!m_bRunning) return false;
    m_bRunning = false;

    if (m_pWorkSheet)
    {
      xls_close_WS(m_pWorkSheet);
      m_pWorkSheet = nullptr;
    }

    if (m_pWorkBook)
    {
      xls_close_WB(m_pWorkBook);
      m_pWorkBook = nullptr;
    }
    return true;
  }

public:
  CApi4HMI()
  {
    m_fault_code_file = "qrc:/fault_code.xls";
  }

  virtual ISelfCheck* GetSelfCheck(ISelfCheck::e_type type) override
  {
    switch (type)
    {
      case ISelfCheck::e_type::Vehicle:
        {
          static CSelfCheckVehicle check;
          return &check;
        }
        break;
      case ISelfCheck::e_type::System:
        {
          static CSelfCheckSystem check;
          return &check;
        }
        break;

      case ISelfCheck::e_type::Sensor:
        {
          static CSelfCheckSensor check;
          return &check;
        }
        break;

      case ISelfCheck::e_type::Algorithm:
        {
          static CSelfCheckAlgorithm check;
          return &check;
        }

      default:
        return nullptr;
    }
  }

  virtual bool GetFaultList(std::vector<CFault>& faults) override
  {
    for (auto it = this->m_module_reprots.begin(); it != this->m_module_reprots.end(); it++)
    {
      if (it->second.moduleStatus != ads_msgs::ads_module_report::moduleStatus_OK)
      {
        CFault fault;

        std::string szKey;
        szKey = std::to_string(it->second.moduleID)
            + "-" + std::to_string(it->second.moduleStatus)
            + "-" + std::to_string(it->second.moduleSubtype);

        if (it->second.deviceID > 0)
        {
          szKey = szKey + "-id";
        }

        auto row = getFaultRow(szKey);
        if (row)
        {
          fault.szCode = it->first;
          fault.szName = reinterpret_cast<char*>(row->cells.cell[2].str);
          if (it->second.deviceID > 0)
          {
            fault.szName = fault.szName + ",id=" + std::to_string(it->second.deviceID);
          }
          fault.szCondition = reinterpret_cast<char*>(row->cells.cell[3].str);
          fault.szHandle = reinterpret_cast<char*>(row->cells.cell[4].str);
          fault.szAction = reinterpret_cast<char*>(row->cells.cell[5].str);
        }
        else
        {
          fault.szCode = it->first;
        }
        faults.push_back(fault);
      }
    }
    if(m_bDebug)
    {
      std::cout << "Total faults=" << faults.size() << std::endl;
    }
    return true;
  }

  virtual std::map<std::string, boost::any> GetProperty(const std::vector<std::string>& items) override
  {
    std::map<std::string, boost::any> values;
    for (auto it = items.begin(); it != items.end(); it++)
    {
      if (it->compare(IApi4HMI::Item_penshui) == 0)
      {
        values[*it] = 123;
      }
      else if (it->compare(IApi4HMI::Item_saopan) == 0)
      {
        values[*it] = 3;
      }
      else if (it->compare(IApi4HMI::Item_shou_sha) == 0)
      {
        values[*it] = 1.07;
      }
      else if (it->compare(IApi4HMI::Item_shikuo_light) == 0)
      {
        values[*it] = 0;
      }
      else if (it->compare(IApi4HMI::Item_yingji_light) == 0)
      {
        values[*it] = 0;
      }
      else if (it->compare(IApi4HMI::Item_jinguang_light) == 0)
      {
        values[*it] = 0;
      }
      else if (it->compare(IApi4HMI::Item_checliang_fault) == 0)
      {
        values[*it] = 'A';
      }
      else if (it->compare(IApi4HMI::Item_yuanguang_light) == 0)
      {
        values[*it] = std::string("Test");
      }
    }
    return values;
  }

  void OnMsg_PointCloud2(const sensor_msgs::PointCloud2& msg)
  {
    m_CloudPointSignal(msg);
  }

  virtual boost::signals2::connection ConnectPointCloud(const std::string& szName, std::function<void(const sensor_msgs::PointCloud2&)> callback) override
  {
    m_Subs[szName] = s_nodeHandles[0]->subscribe(szName, 10, &CApi4HMI::OnMsg_PointCloud2, this);
    return m_CloudPointSignal.connect(callback);
  }

  virtual bool UpdateMapInfo(const std::string& strMapPath) override
  {
    (void)strMapPath;
    return true;
  }

  virtual bool StartAutoDrive() override
  {
    return true;
  }

  virtual bool StopAutoDrive() override
  {
    return true;
  }

  virtual boost::signals2::connection InstallEventCallback(std::function<int(CEventReport&)> callback)override
  {
    return m_eventSignal.connect(callback);
  }

  virtual bool StartWithHost(IHostApi4HMI* host, const std::string& szFile, int iDebug = 0) override
  {
    this->m_bDebug = iDebug;
    m_hostApi = host;
    if (!szFile.empty())
    {
      m_fault_code_file = szFile;
    }

    return Init();
  }
};
}

namespace dbAds
{
#define DECLARE_STATE_ITEM(name)   const std::string IApi4HMI::Item_##name = #name;
DECLARE_STATE_ITEM(shou_sha);
DECLARE_STATE_ITEM(shikuo_light);
DECLARE_STATE_ITEM(jinguang_light);
DECLARE_STATE_ITEM(yuanguang_light);
DECLARE_STATE_ITEM(yingji_light);
DECLARE_STATE_ITEM(checliang_fault);
DECLARE_STATE_ITEM(saopan);
DECLARE_STATE_ITEM(penshui);
#undef DECLARE_STATE_ITEM

IApi4HMI* getApi4HMI()
{
  static CApi4HMI api;
  return &api;
}
}

#if 0
void Api4HMI_TestCase(ros::NodeHandle* lpnode)
{
  class CHostApi4HMI :public IHostApi4HMI
  {
  public:
    dbAds::IApi4HMI* m_lpApi = nullptr;
    ros::NodeHandle* m_lpnode = nullptr;

    virtual void getNodeHandle(std::vector<ros::NodeHandle*>& nhs) override
    {
      nhs.push_back(m_lpnode);
    }

    std::map<std::string, ros::Subscriber> m_Subs;
    std::map<std::string, ros::Publisher> m_Pubs;

    void OnMsg_ads_hmi_command(const ads_msgs::ads_hmi_command& msg)
    {
      std::cout << __FUNCTION__ << "(" << msg.wRunState << ")" << std::endl;
    }

    CHostApi4HMI(ros::NodeHandle*lpnode)
    {
      m_lpnode = lpnode;

      m_lpApi = dbAds::getApi4HMI();
      m_lpApi->StartWithHost(this, "", 1);

      m_lpApi->InstallEventCallback([this](dbAds::IApi4HMI::CEventReport& report)
      {
        std::cout << "report.szCode:" << report.szCode << std::endl;

        std::vector<dbAds::IApi4HMI::CFault> faults;
        m_lpApi->GetFaultList(faults);
        for (auto it = faults.begin(); it != faults.end(); it++)
        {
          std::cout << *it;
        }

        return 1;
      });

      std::string topic = "/ads_hmi_command";
      m_Subs[topic] = m_lpnode->subscribe(topic, 10, &CHostApi4HMI::OnMsg_ads_hmi_command, this);

      topic = "/ads_module_report";
      m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_module_report>(topic, 2);

      topic = "/ads_control_WheelPositionRepor";
      m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_control_WheelPositionReport>(topic, 10);

      topic = "/ads_control_brakereport";
      m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_control_brakereport>(topic, 10);

      topic = "/ads_control_fuel_level_report";
      m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_control_fuel_level_report>(topic, 10);

      topic = "/ads_control_steering_report";
      m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_control_steering_report>(topic, 10);

      topic = "/ads_control_throttle_report";
      m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_control_throttle_report>(topic, 10);

      topic = "/ads_device_report";
      m_Pubs[topic] = m_lpnode->advertise<ads_msgs::ads_device_report>(topic, 10);
    }

    void run()
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
        while(check->GetResult() == ISelfCheck::Processing)
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

        auto values = m_lpApi->GetProperty({ dbAds::IApi4HMI::Item_shou_sha
                                             , dbAds::IApi4HMI::Item_shikuo_light
                                             , dbAds::IApi4HMI::Item_jinguang_light
                                             , dbAds::IApi4HMI::Item_yuanguang_light
                                             , dbAds::IApi4HMI::Item_yingji_light
                                             , dbAds::IApi4HMI::Item_checliang_fault
                                             , dbAds::IApi4HMI::Item_saopan
                                             , dbAds::IApi4HMI::Item_penshui
                                           }
                                           );

        for(auto it = values.begin(); it != values.end(); it++)
        {
          if(it->second.type() == typeid (int))
          {
            auto value = boost::any_cast<int>(it->second);
            std::cout << it->first << "=" << value << std::endl;
          }
          else if(it->second.type() == typeid (float))
          {
            auto value = boost::any_cast<float>(it->second);
            std::cout << it->first << "=" << value << std::endl;
          }
          else if(it->second.type() == typeid (double))
          {
            auto value = boost::any_cast<double>(it->second);
            std::cout << it->first << "=" << value << std::endl;
          }
          else if(it->second.type() == typeid (std::string))
          {
            auto value = boost::any_cast<std::string>(it->second);
            std::cout << it->first << "=\"" << value << "\"" << std::endl;
          }
          else
          {
            std::cout << it->first << " with unsuportd type:" << it->second.type().name() << std::endl;
          }
        }

        std::this_thread::sleep_for(std::chrono::seconds(10));
      }
    }
  } s_HostApi(lpnode);

  s_HostApi.run();
}
#else
void Api4HMI_TestCase(ros::NodeHandle* lpnode)
{

}
#endif
