#include "dbAdsApi4HMI.h"

#include <memory>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "std_msgs/String.h"
#include "ads_msgs/ads_ad_command.h"
#include "ads_msgs/ads_ad_report.h"
#include "ads_msgs/ads_module_report.h"

#include "ads_msgs/ads_control_WheelPositionReport.h"
#include "ads_msgs/ads_control_brakereport.h"
#include "ads_msgs/ads_control_fuel_level_report.h"
#include "ads_msgs/ads_control_steering_report.h"
#include "ads_msgs/ads_control_throttle_report.h"
#include "ads_msgs/ads_control_light.h"
#include "ads_msgs/ads_control_sweep.h"
#include "ads_msgs/ads_control_systemmode.h"

#include <xls.h>
using namespace xls;
using namespace dbAds;

namespace{
    class CSelfCheckChassis : public ISelfCheck
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
                for(m_progress = 0; m_progress < 100; m_progress += 10)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
                for(m_progress = 0; m_progress < 100; m_progress += 10)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
                for(m_progress = 0; m_progress < 100; m_progress += 10)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
                for(m_progress = 0; m_progress < 100; m_progress += 10)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

    class CApi4HMI : public dbAds::IApi4HMI
    {
    private:
        int m_bDebug = 0;
        bool m_bRunning = false;
        std::vector<std::string> m_cared_property;
		
        std::string m_fault_code_file;
        std::unique_ptr<ISelfCheck> m_check_System;
        std::unique_ptr<ISelfCheck> m_check_Sensor;
        std::unique_ptr<ISelfCheck> m_check_Algorithm;
        std::unique_ptr<ISelfCheck> m_check_Chassis;

        boost::signals2::signal<int(CEventReport&)> m_eventSignal;
        boost::signals2::signal<void(const sensor_msgs::PointCloud2&)> m_CloudPointSignal;

        std::vector<ros::NodeHandle*> s_nodeHandles;

        std::map<std::string, ros::Subscriber> m_Subs;
        std::map<std::string, ros::Publisher> m_Pubs;

        IHostApi4HMI* m_hostApi = nullptr;
        xlsWorkBook* m_pWorkBook = nullptr;
        xlsWorkSheet* m_pWorkSheet = nullptr;

        std:: recursive_mutex  m_mutex;
        std::condition_variable m_cv;
        //Below member is protected
        ads_msgs::ads_control_WheelPositionReport m_msg_ads_control_WheelPositionReport;
        ads_msgs::ads_control_brakereport m_msg_ads_control_brakereport;
        ads_msgs::ads_control_fuel_level_report m_msg_ads_control_fuel_level_report;
        ads_msgs::ads_control_steering_report m_msg_ads_control_steering_report;
        ads_msgs::ads_control_throttle_report m_msg_ads_control_throttle_report;
        ads_msgs::ads_ad_report m_msg_ads_ad_report;

        ads_msgs::ads_control_light m_msg_ads_control_light;
        ads_msgs::ads_control_sweep m_msg_ads_control_sweep;
        ads_msgs::ads_control_systemmode m_msg_ads_control_systemmode;

        std::map<int , std::chrono::system_clock::time_point> m_module_beat;
        std::map<std::string, ads_msgs::ads_module_report> m_module_reprots;

    private:
        void OnMsg_ads_module_report(const ads_msgs::ads_module_report& msg)
        {
            std::string szKey;
            {
                std::lock_guard<std:: recursive_mutex> lock(m_mutex);
                m_module_beat[msg.moduleID] = std::chrono::system_clock::now();

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
            }

            if (msg.moduleStatus == ads_msgs::ads_module_report::moduleStatus_Critical)
            {
                ads_msgs::ads_ad_command msg_cmd;
                msg_cmd.action = ads_msgs::ads_ad_command::action_Stop;
                m_Pubs["/ads_ad_command"].publish(msg_cmd);
                ros::spinOnce();
            }

            CEventReport report;
            report.m_ItemName = Item_module_state;
            report.m_value = szKey;
            m_eventSignal(report);
        }

        void OnMsg_ads_control_WheelPositionReport(const ads_msgs::ads_control_WheelPositionReport& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_WheelPositionReport = msg;
        }

        void OnMsg_ads_control_brakereport(const ads_msgs::ads_control_brakereport& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_brakereport = msg;
        }


        void OnMsg_ads_control_fuel_level_report(const ads_msgs::ads_control_fuel_level_report& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_fuel_level_report = msg;
        }


        void OnMsg_ads_control_steering_report(const ads_msgs::ads_control_steering_report& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_steering_report = msg;
        }

        void OnMsg_ads_control_throttle_report(const ads_msgs::ads_control_throttle_report& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_throttle_report = msg;
        }

        void OnMsg_ads_ad_report(const ads_msgs::ads_ad_report& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_ad_report = msg;
        }

        void OnMsg_ads_control_light(const ads_msgs::ads_control_light& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_light = msg;
        }

        void OnMsg_ads_control_sweep(const ads_msgs::ads_control_sweep& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_sweep = msg;
        }

        void OnMsg_ads_control_systemmode(const ads_msgs::ads_control_systemmode& msg)
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
            m_msg_ads_control_systemmode = msg;
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

            std::string topic = "/ads_ad_command";
            m_Pubs[topic] = s_nodeHandles[0]->advertise<ads_msgs::ads_ad_command>(topic, 2);

            //ros::message_traits::definition<ads_msgs::ads_ad_command>();

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

            topic = "/ads_ad_report";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_ad_report, this);

            topic = "/ads_control_light";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_light, this);

            topic = "/ads_control_sweep";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_sweep, this);

            topic = "/ads_control_systemmode";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_ads_control_systemmode, this);

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
            m_fault_code_file = "./fault_code.xls";
        }

        virtual ISelfCheck* GetSelfCheck(ISelfCheck::e_type type) override
        {
            switch (type)
            {
            case ISelfCheck::e_type::System:
            {
                if(!m_check_System)
                {
                    m_check_System.reset(new CSelfCheckSystem());
                }
                return m_check_System.get();
            }
            break;

            case ISelfCheck::e_type::Sensor:
            {
                if(!m_check_Sensor)
                {
                    m_check_Sensor.reset(new CSelfCheckSensor());
                }
                return m_check_Sensor.get();
            }
            break;

            case ISelfCheck::e_type::Algorithm:
            {
                if(!m_check_Algorithm)
                {
                    m_check_Algorithm.reset(new CSelfCheckAlgorithm());
                }
                return m_check_Algorithm.get();
            }
            case ISelfCheck::e_type::Chassis:
            {
                if(!m_check_Chassis)
                {
                    m_check_Chassis.reset(new CSelfCheckChassis());
                }
                return m_check_Chassis.get();
            }

            default:
                return nullptr;
            }
        }

        virtual bool GetFaultList(std::vector<CFault>& faults) override
        {
            std::lock_guard<std:: recursive_mutex> lock(m_mutex);
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
/*
            ads_msgs::ads_control_WheelPositionReport m_msg_ads_control_WheelPositionReport;
            ads_msgs::ads_control_brakereport m_msg_ads_control_brakereport;
            ads_msgs::ads_control_fuel_level_report m_msg_ads_control_fuel_level_report;
            ads_msgs::ads_control_steering_report m_msg_ads_control_steering_report;
            ads_msgs::ads_control_throttle_report m_msg_ads_control_throttle_report;
            ads_msgs::ads_ad_report m_msg_ads_ad_report;
            ads_msgs::ads_control_light m_msg_ads_control_light;

            m_msg_ads_control_light.LeftTurnLight   #light for turn
            m_msg_ads_control_light.RightTurnLight
            m_msg_ads_control_light.LowBeamLight    #light for lighting
            m_msg_ads_control_light.HighBeamLight
            m_msg_ads_control_fuel_level_report.rsoc //dian liang%
*/

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
                    values[*it] = m_msg_ads_control_light.BrakeLight;
                }
                else if (it->compare(IApi4HMI::Item_shikuo_light) == 0)
                {
                    values[*it] = m_msg_ads_control_light.WidthLight;      //light for show status
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
                else if (it->compare(IApi4HMI::Item_car_state) == 0)
                {
                    values[*it] = static_cast<ads_ad_report::_carState_type>(m_msg_ads_ad_report.carState);
                }
                else if (it->compare(IApi4HMI::Item_shui_liang) == 0)
                {
                    values[*it] = 10;
                }
                else if (it->compare(dbAds::IApi4HMI::Item_dang_wei) == 0)
                {
                    values[*it] = 1;
                }
                else
                {
                    std::cout << "Unknown Property:" << *it << std::endl;
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

        virtual bool StartAutoDrive(unsigned int action = ads_ad_command::action_SweepTo,
                                    double x = 0.0, double y = 0.0, double z = 0.0) override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = action;
            msg.lat = x;
            msg.lon = y;

            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }

        virtual bool StopAutoDrive() override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = ads_ad_command::action_Stop;

            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }


        virtual boost::signals2::connection InstallEventCallback(std::function<int(CEventReport&)> callback,
                                                                 const std::vector<std::string>& property)override
        {
            m_cared_property = property;
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

    DECLARE_STATE_ITEM(dang_wei);
    DECLARE_STATE_ITEM(shui_liang);

    DECLARE_STATE_ITEM(car_state);
    DECLARE_STATE_ITEM(module_state);

#undef DECLARE_STATE_ITEM

    IApi4HMI* getApi4HMI()
    {
        static CApi4HMI api;
        return &api;
    }
}

#if 1
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

        void OnMsg_ads_ad_command(const ads_msgs::ads_ad_command& msg)
        {
            std::cout << __FUNCTION__ << "(" << msg.action << ")" << std::endl;
        }

        CHostApi4HMI(ros::NodeHandle*lpnode)
        {
            m_lpnode = lpnode;

            m_lpApi = dbAds::getApi4HMI();
            m_lpApi->StartWithHost(this, "", 1);

            std::vector<std::string> cared_property = { dbAds::IApi4HMI::Item_shou_sha
                                                        , dbAds::IApi4HMI::Item_shikuo_light
                                                        , dbAds::IApi4HMI::Item_jinguang_light
                                                        , dbAds::IApi4HMI::Item_yuanguang_light
                                                        , dbAds::IApi4HMI::Item_yingji_light
                                                        , dbAds::IApi4HMI::Item_checliang_fault
                                                        , dbAds::IApi4HMI::Item_saopan
                                                        , dbAds::IApi4HMI::Item_penshui
                                                        , dbAds::IApi4HMI::Item_car_state
                                                        , dbAds::IApi4HMI::Item_module_state
                                                        , dbAds::IApi4HMI::Item_shui_liang
                                                        , dbAds::IApi4HMI::Item_dang_wei
                                                       };
            m_lpApi->InstallEventCallback([this](dbAds::IApi4HMI::CEventReport& report)
            {
                if(dbAds::IApi4HMI::Item_module_state == report.m_ItemName)
                {
                    std::cout << "report[" << report.m_ItemName << "]="
                              << boost::any_cast<std::string>(report.m_value)
                              << std::endl;

                    std::vector<dbAds::IApi4HMI::CFault> faults;
                    m_lpApi->GetFaultList(faults);
                    for (auto it = faults.begin(); it != faults.end(); it++)
                    {
                        std::cout << *it;
                    }
                }
                return 1;
            }, cared_property);

            std::string topic = "/ads_ad_command";
            m_Subs[topic] = m_lpnode->subscribe(topic, 10, &CHostApi4HMI::OnMsg_ads_ad_command, this);

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
                                                     , dbAds::IApi4HMI::Item_car_state
                                                     , dbAds::IApi4HMI::Item_shui_liang
                                                     , dbAds::IApi4HMI::Item_dang_wei
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
                    else if(it->second.type() == typeid (ads_msgs::ads_ad_report::_carState_type))
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
    } s_HostApi(lpnode);

    s_HostApi.run();
}
#else
void Api4HMI_TestCase(ros::NodeHandle* lpnode)
{

}
#endif
