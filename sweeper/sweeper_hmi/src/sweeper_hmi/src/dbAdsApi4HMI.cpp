#include "dbAdsApi4HMI.h"

#include <memory>
#include <iostream>
#include <fstream>
		
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <regex>

#include "jsoncpp/json/json.h"

#include "std_msgs/String.h"

#include "ads_msgs/ads_ad_command.h"
#include "ads_msgs/ads_site_data_path.h"
#include "ads_msgs/ads_ad_report.h"
#include "ads_msgs/ads_module_report.h"
#include "ads_msgs/ads_str_array.h"
#include "ads_msgs/ads_int2int_data.h"
#include "ads_msgs/ads_sweeper_control_id.h"
#include "ads_msgs/Status.h"
#include "ads_msgs/ads_ins_data.h"
#include "ads_msgs/ads_PlanningData4Debug.h"

#include "sweep_msgs/ThrottleReport.h"
#include "sweep_msgs/BrakeReport.h"
#include "sweep_msgs/SteeringReport.h"
#include "sweep_msgs/SystemMode.h"
#include "sweep_msgs/Light.h"
#include "sweep_msgs/FuelLevelReport.h"

#include "ads_msgs/srv_get_site_job_info.h"

#include <xls.h>

using namespace xls;
using namespace dbAds;
using namespace ads_msgs;

static bool g_bEnableFaultHandle = (nullptr != getenv("ENABLE_FAULT_HANDLE"));

#define _CREATE_FAULT_SH_
#define _TEST_ENABLE_SELF_CHECK_
//#define _TEST_ENABLE_GETPROPERTY_

namespace {
    struct 
    {
        std::set<std::string> m_ignore_fault_list;
        bool m_charge_fault = false;
        std::string m_fault_code_file{"./fault_code.xls"};
        int m_debug = 0;
    } s_my_config;

    inline int GetIntKey(int ID, int STATUS, int TYPE, int DEVICE = 0)
    {
        return ((ID << 24) | (STATUS << 16) | (TYPE << 8) | DEVICE);
    }

    class IFaultProvider
    {
    public:
       enum e_type
        {
            None,
            Chassis,
            System,
            Sensor,
            Algorithm,
            DisableAd,
            StopAd,
        };

         virtual bool QueryFaultAmount(e_type type, int& iCritical, int& iWarning) = 0;
         virtual ~IFaultProvider(){}
    };

    class CModule_report_item_Comp
    {
    public:
        bool operator()(const ads_msgs::ads_module_report_item& left, const ads_msgs::ads_module_report_item& right)
        {
            if(left.moduleID != right.moduleID)
            {
                return left.moduleID < right.moduleID;
            } 

            if(left.moduleStatus != right.moduleStatus)
            {
                return left.moduleStatus < right.moduleStatus;
            }

            if(left.moduleSubtype != right.moduleSubtype)
            {
                return left.moduleSubtype < right.moduleSubtype;
            }

            return left.deviceID < right.deviceID;
        }
    };

    class CSelfCheckImp : public ISelfCheck
    {
    private:
        std::thread m_th;
        int m_progress = 0;
        ISelfCheck::e_result m_result = ISelfCheck::Unknown;
        bool m_bStop = true;
        IFaultProvider* m_lpProvider = nullptr;
        ISelfCheck::e_type m_type;
        int m_FaultAmountInit[2] = {0 , 0};

    public:
        explicit CSelfCheckImp(IFaultProvider* lpProvider, ISelfCheck::e_type type)
        {
            m_lpProvider = lpProvider;
            m_type = type;
        }

        virtual bool Start() override
        {
            if (!m_bStop) return true;
            m_bStop = false;

            static std::map<ISelfCheck::e_type, IFaultProvider::e_type> type_map =
            {
                {ISelfCheck::Sensor,    IFaultProvider::Sensor},
                {ISelfCheck::Chassis,   IFaultProvider::Chassis},
                {ISelfCheck::Algorithm, IFaultProvider::Algorithm},
                {ISelfCheck::System,    IFaultProvider::System},
            };

            if(type_map.find(m_type) == type_map.end()) 
            {
                m_result = ISelfCheck::Failure;
                return false;
            }

            // m_lpProvider->QueryFaultAmount(type_map[m_type], m_FaultAmountInit[0], m_FaultAmountInit[1]);
            // if ((m_FaultAmountInit[0] + m_FaultAmountInit[1]) == 0)
            // {
            //     m_progress = 100;
            //     m_result = ISelfCheck::Pass;
            //     return true;
            // }

            m_result = ISelfCheck::Processing;

            m_th = std::thread([this](IFaultProvider::e_type type) 
            {
                auto tp_start = std::chrono::system_clock::now();

                for (m_progress = 0; m_progress < 100; m_progress += 1)
                {
                    if(g_bEnableFaultHandle)
                    {
                        int iFaultAmount[2] = { 0 , 0 };
                        m_lpProvider->QueryFaultAmount(type, iFaultAmount[0], iFaultAmount[1]);
                        if((iFaultAmount[0] + iFaultAmount[1]) == 0)
                        {
                            //确保UI显示足够时间
                            if(std::chrono::system_clock::now() - tp_start > std::chrono::seconds(4))
                            //if(m_progress >= 50)
                            {
                                m_result = ISelfCheck::Pass;
                                break;                                
                            }
                        }
                    }
                    
                    if((type == IFaultProvider::System) && g_bEnableFaultHandle)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(400));
                    }
                    else
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    if (m_bStop) break;
                }

                if(!g_bEnableFaultHandle)
                {
                    m_result = ISelfCheck::Pass;
                }

                if (m_bStop)
                {
                    m_result = ISelfCheck::Unknown;
                }
                else if(m_result != ISelfCheck::Pass)
                {
                    m_result = ISelfCheck::Failure;
                }
            }, type_map[m_type]);
            
            return true;
        }

        virtual bool Stop() override
        {
            m_bStop = true;
            if (m_th.joinable())
            {
                m_th.join();
            }
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
 
    class CFaultItemCategoryInfo
    {
    public:
        CFaultItemCategoryInfo(int post, int ad_init, int ad)
            : m_category_post(post), m_category_ad_init(ad_init), m_category_ad(ad)
        {

        }
    public:
        int m_category_post; //1:IFaultProvider::Chassis, 2:System, 3:Sensor, 4:Algorithm
        int m_category_ad_init;
        int m_category_ad;
    };

    class CFaultDataBase
    {
    private:
        xlsWorkBook* m_pWorkBook = nullptr;
        xlsWorkSheet* m_pWorkSheet = nullptr;

        int m_colFaultModule = 0;
        int m_colFaultCode = 1;
        int m_colCategoryPost = 2;
        int m_colCategoryAdInit = 3;
        int m_colCategoryAd = 4;
        int m_colIDAmount = 5;
        int m_colFaultName = 6;
        int m_colCondition = 7;
        int m_colHandle = 8;
        int m_colAction = 9;

        std::map<int, CFaultItemCategoryInfo> m_CategoryInfors;
        std::string m_fault_code_file;

    public:
        CFaultDataBase(const std::string& file)
        {
            m_fault_code_file = file;
            if (nullptr == m_pWorkBook)
            {
                m_pWorkBook = xls_open(m_fault_code_file.c_str(), "UTF-8");
                if (nullptr == m_pWorkBook)
                {
                    std::cerr << "file is not excel" << std::endl;
                }
                else
                {
                    auto index = 0;
                    for (; index < m_pWorkBook->sheets.count; index++)
                    {
                        auto lpSheet = m_pWorkBook->sheets.sheet + index;
                        if (std::strcmp("故障代码定义", reinterpret_cast<char*>(lpSheet->name)) == 0)
                        {
                            m_pWorkSheet = xls_getWorkSheet(m_pWorkBook, index);
                            xls_parseWorkSheet(m_pWorkSheet);
                            break;
                        }
                    }
                }
            }

            if (nullptr != m_pWorkSheet)
            {
                std::map<std::string, int*> name2col =
                {
                    {"系统部件",            &m_colFaultModule},
                    {"故障代码",            &m_colFaultCode},
                    {"开机自检",            &m_colCategoryPost},
                    {"自动驾驶前置条件",     &m_colCategoryAdInit},
                    {"自动驾驶模式",         &m_colCategoryAd},
                    {"ID数量",              &m_colIDAmount},
                    {"故障名称",            &m_colFaultName},
                    {"故障判定条件",        &m_colCondition},
                    {"处理方式",            &m_colHandle},
                    {"故障对策",            &m_colAction},
                };

                for (auto& it : name2col)
                {
                    *(it.second) = -1;
                    for (int col = 0; col < name2col.size(); col++)
                    {
                        xlsCell* cell = xls_cell(m_pWorkSheet, 0, col);
                        if ((nullptr != cell) && (nullptr != cell->str))
                        {
                            if (it.first.compare(reinterpret_cast<char*>(cell->str)) == 0)
                            {
                                *(it.second) = col;
                                std::cout << it.first << ":" << *(it.second) << std::endl;
                            }
                        }
                    }
                    if (-1 == *(it.second))
                    {
                        std::cout << it.first << " can not be found!" << std::endl;
                    }
                }
            }

            if(nullptr != m_pWorkSheet)
            {
                for (int index = 1; ; index++)
                {
                    xlsCell* cell = xls_cell(m_pWorkSheet, index, m_colFaultCode);

                    if (nullptr == cell) break;
                    if (nullptr == cell->str) break;

                    int moduleID, moduleStatus, moduleSubtype, devices;
                    auto result = std::sscanf(reinterpret_cast<char*>(cell->str), "%d-%d-%d-%d",
                        &moduleID, &moduleStatus, &moduleSubtype, &devices);
                    switch (result)
                    {
                    default:
                        std::cout << "wrong fault code:" << reinterpret_cast<char*>(cell->str) << std::endl;
                        break;

                    case 3:
                    case 4:
                    {
                        if(m_colCategoryPost > 0)
                        {
                            CFaultItemCategoryInfo info(IFaultProvider::None, 0, 0);
                            char* lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, index, m_colCategoryPost)->str);
                            if(lp)
                            {
                                if(std::string("System") == lp)
                                {
                                    info.m_category_post = IFaultProvider::System;
                                }
                                else if(std::string("Chassis") == lp)
                                {
                                    info.m_category_post = IFaultProvider::Chassis;
                                }
                                else if(std::string("Sensor") == lp)
                                {
                                    info.m_category_post = IFaultProvider::Sensor;
                                }
                                else if(std::string("Algorithm") == lp)
                                {
                                    info.m_category_post = IFaultProvider::Algorithm;
                                }
                                else 
                                {
                                    info.m_category_post = IFaultProvider::None;
                                }
                            }

                            lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, index, m_colCategoryAdInit)->str);
                            if(lp)
                            {
                                info.m_category_ad_init = std::strtol(lp, nullptr, 0);
                            }

                            lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, index, m_colCategoryAd)->str);
                            if(lp)
                            {
                                info.m_category_ad = std::strtol(lp, nullptr, 0);
                            }

                            m_CategoryInfors.emplace(GetIntKey(moduleID, moduleStatus, moduleSubtype), 
                                CFaultItemCategoryInfo(info.m_category_post, info.m_category_ad_init, info.m_category_ad)
                                );
                        }
                    }
                    break;
                    }
                }
            }
        }

        CFaultItemCategoryInfo* GetItemCategory(int moduleID, int moduleStatus, int moduleSubtype)
        {
            auto it = m_CategoryInfors.find(GetIntKey(moduleID, moduleStatus, moduleSubtype));
            if(it != m_CategoryInfors.end())
            {
                return &(it->second);
            }

            return nullptr;
        }

        bool FillCFaultInfo(const ads_module_report_item& item, IApi4HMI::CFaultInfo& fault)
        {
            std::string szKey = std::string("^")
                + std::to_string(item.moduleID)
                + "-" + std::to_string(item.moduleStatus)
                + "-" + std::to_string(item.moduleSubtype)
                + "(-id){0,1}";
            if (m_pWorkSheet)
            {
                std::regex self_regex(szKey);

                for (int r = 1; r <= m_pWorkSheet->rows.lastrow; r++)
                {
                    xlsCell* cell = xls_cell(m_pWorkSheet, r, m_colFaultCode);
                    if (nullptr == cell) break;
                    if (nullptr == cell->str) break;
                    if (std::regex_match(reinterpret_cast<char*>(cell->str), self_regex))
                    {
                        auto lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colFaultCode)->str);
                        if(lp)
                        {
                            fault.szCode = std::to_string(item.moduleID)
                                            + "-" + std::to_string(item.moduleStatus)
                                            + "-" + std::to_string(item.moduleSubtype);
                            if(item.deviceID > 0)
                            {
                                fault.szCode = fault.szCode + "-" + std::to_string(item.deviceID);
                            }
                        }

                        lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colFaultName)->str);
                        if(lp)
                        {
                            fault.szName = lp;
                            if(item.deviceID > 0)
                            {
                                fault.szName = fault.szName + "(" + std::to_string(item.deviceID) + ")";
                            }
                        }

                        lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colCondition)->str);
                        if(lp)
                        {
                            fault.szCondition = lp;
                        }

                        lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colHandle)->str);
                        if(lp)
                        {
                            fault.szHandle = lp;
                        }

                        lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colAction)->str);
                        if(lp)
                        {
                            fault.szAction = lp;
                        }


                        lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colCategoryPost)->str);
                        if(lp)
                        {
                            fault.szCategory = fault.szCategory + ":" + lp;
                        }

                        lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colCategoryAdInit)->str);
                        if(lp)
                        {
                            fault.szCategory = fault.szCategory + "," + lp;
                        }
                        lp = reinterpret_cast<char*>(xls_cell(m_pWorkSheet, r, m_colCategoryAd)->str);
                        if(lp)
                        {
                            fault.szCategory = fault.szCategory + "," + lp;
                        }

                        return true;
                    }
                }
            }
            std::cout << "Did not find information for \"" << szKey << "\"" << std::endl;
            return false;
        }

        bool GetDefaultFaultList(std::set<ads_module_report_item, CModule_report_item_Comp>& list)
        {
            if (nullptr == m_pWorkSheet) return false;

#ifdef _CREATE_FAULT_SH_
            std::string s = R"(rostopic pub -1 /ads_module_report ads_msgs/ads_module_report "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
reports:
- moduleID: %d
  moduleStatus: %d
  moduleSubtype: %d
  deviceID: %d"

sleep 2s

            )";
            //'{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

            std::ofstream of("fault.sh", std::ios::trunc);
            of << "#!/bin/bash" << std::endl;

            char szBuffer[1000];
#endif

            for (int r = 1; r <= m_pWorkSheet->rows.lastrow; r++)
            {
                xlsCell* cell = xls_cell(m_pWorkSheet, r, m_colFaultCode);
                if (nullptr == cell) break;
                if (nullptr == cell->str) break;

                int moduleID, moduleStatus, moduleSubtype;
                auto result = std::sscanf(reinterpret_cast<char*>(cell->str), "%d-%d-%d", &moduleID, &moduleStatus, &moduleSubtype);
                if(3 != result)
                {
                    std::cout << "Wrong fault code format:" << reinterpret_cast<char*>(cell->str) << std::endl;
                }
                else
                {
                    int devices = 0;
                    if(m_colIDAmount >= 0)
                    {
                        xlsCell* cellAmount = xls_cell(m_pWorkSheet, r, m_colIDAmount);
                        if ((nullptr != cellAmount) && (nullptr != cellAmount->str))
                        {
                            devices = std::strtol(reinterpret_cast<char*>(cellAmount->str), nullptr, 10);
                        }
                    }

                    
                    if(devices > 0)
                    {
                        for (int device = 1; device <= devices; device++)
                        {
                            ads_module_report_item item;
                            item.moduleStatus = moduleStatus;
                            item.moduleID = moduleID;
                            item.moduleSubtype = moduleSubtype;
                            item.deviceID = device;

                            std::string szOut;
                            szOut = std::to_string(item.moduleID)
                                + "-" + std::to_string(item.moduleStatus)
                                + "-" + std::to_string(item.moduleSubtype);

                            if (item.deviceID > 0)
                            {
                                szOut = szOut + "-" + std::to_string(item.deviceID);
                            }

                            if(s_my_config.m_ignore_fault_list.find(szOut) == s_my_config.m_ignore_fault_list.end())
                            {
                                list.insert(item);
#ifdef _CREATE_FAULT_SH_
                                sprintf(szBuffer, s.c_str(), item.moduleID, ads_module_report_item::moduleStatus_OK, item.moduleSubtype, item.deviceID);
                                of << szBuffer << std::endl;
#endif                            
                            } 
                        }
                    }
                    else
                    {
                            ads_module_report_item item;
                            item.moduleStatus = moduleStatus;
                            item.moduleID = moduleID;
                            item.moduleSubtype = moduleSubtype;
                            item.deviceID = 0;

                            std::string szOut;
                            szOut = std::to_string(item.moduleID)
                                + "-" + std::to_string(item.moduleStatus)
                                + "-" + std::to_string(item.moduleSubtype);

                            if (item.deviceID > 0)
                            {
                                szOut = szOut + "-" + std::to_string(item.deviceID);
                            }

                            if(s_my_config.m_ignore_fault_list.find(szOut) == s_my_config.m_ignore_fault_list.end())
                            {
                                list.insert(item);
#ifdef _CREATE_FAULT_SH_
                                sprintf(szBuffer, s.c_str(), item.moduleID, ads_module_report_item::moduleStatus_OK, item.moduleSubtype, item.deviceID);
                                of << szBuffer << std::endl;
#endif
                            }
                    }
                    
                }
            }

            return true;
        }

        ~CFaultDataBase()
        {
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
        }
    };

    class CApi4HMI : public dbAds::IApi4HMI, public IFaultProvider
    {
    private:
        bool m_bRunning = false;
        std::vector<std::string> m_cared_property;

        std::map<ISelfCheck::e_type, std::shared_ptr<ISelfCheck>> m_SelfCheckPool;

        std::unique_ptr<CFaultDataBase> m_lpFaultDataBase;

        boost::signals2::signal<int(CEventReport&)> m_eventSignal;
        boost::signals2::signal<void(const sensor_msgs::PointCloud2&)> m_CloudPointSignal;

        std::vector<ros::NodeHandle*> s_nodeHandles;

        std::map<std::string, ros::Subscriber> m_Subs;
        std::map<std::string, ros::Publisher> m_Pubs;

        IHostApi4HMI* m_hostApi = nullptr;

        std::recursive_mutex  m_mutex;
        std::condition_variable m_cv;

        //Below member is protected by m_mutex
        sweep_msgs::ThrottleReport m_ThrottleReport;
        sweep_msgs::BrakeReport m_BrakeReport;
        sweep_msgs::SteeringReport m_SteeringReport;
        sweep_msgs::SystemMode m_SystemMode;
        sweep_msgs::Light m_Light;
        sweep_msgs::FuelLevelReport m_FuelLevelReport;
        ads_msgs::ads_ad_report m_msg_ads_ad_report;
        ads_msgs::Status m_chasiss_status;
        ads_msgs::ads_ins_data m_ads_ins_data;
        ads_msgs::ads_PlanningData4Debug m_planningDataDebug;

        //std::map<int, std::chrono::system_clock::time_point> m_module_beat;
        std::set<ads_msgs::ads_module_report_item, CModule_report_item_Comp> m_module_reports;

        //---------------------------------------------------

    private:

        void OnMsg_StringCallback(const std_msgs::String& msg)
        {
            std::cout << __FUNCTION__ << "(\"" << msg.data << "\")" << std::endl;
            if(msg.data == "dump_fault")
            {
                std::vector<dbAds::IApi4HMI::CFaultInfo> faults;
                GetFaultList(faults);
                for (auto it = faults.begin(); it != faults.end(); it++)
                {
                    std::cout << *it << std::endl;
                }
            }
            else if(msg.data == "clear_fault")
            {
                std::lock_guard<std::recursive_mutex> lock(m_mutex);
                m_module_reports.clear();
            }
        }

        void OnMsg_ads_module_report(const ads_msgs::ads_module_report& msgs)
        {
            if(!g_bEnableFaultHandle) 
            {
                return;
            }
            bool bChanged = false;
            uint32_t _count  = 0;
            std::string szCritical, szWarning;
            {
                std::lock_guard<std::recursive_mutex> lock(m_mutex);
                for (auto& msg : msgs.reports)
                {
                    //m_module_beat[msg.moduleID] = std::chrono::system_clock::now();

                    std::string szOut;
                    szOut = std::to_string(msg.moduleID)
                        + "-" + std::to_string(msg.moduleStatus)
                        + "-" + std::to_string(msg.moduleSubtype);

                    if (msg.deviceID > 0)
                    {
                        szOut = szOut + "-" + std::to_string(msg.deviceID);
                    }

                    if(s_my_config.m_ignore_fault_list.find(szOut) != s_my_config.m_ignore_fault_list.end())
                    {
                        continue;
                    } 

                    switch (msg.moduleStatus)
                    {
                    default:
                        continue;

                    case ads_msgs::ads_module_report_item::moduleStatus_Info_Online:
                    /*
                    {
                        for (auto it = m_module_reports.begin(); it != m_module_reports.end(); it++)
                        {
                            if (msg.moduleID == it->moduleID)
                            {
                                //m_module_reports.erase(it);
                            }
                        }
                    }
                    break;
                    */

                    case ads_msgs::ads_module_report_item::moduleStatus_OK:
                    {
                        if(!bChanged) {_count = m_module_reports.size();}
                        for (auto it = m_module_reports.begin(); it != m_module_reports.end(); it++)
                        {
                            if (msg.moduleID == it->moduleID)
                            {
                                if (msg.moduleSubtype == 0)
                                {
                                    m_module_reports.erase(it);
                                }
                                else if (msg.moduleSubtype == it->moduleSubtype)
                                {
                                    if (msg.deviceID == 0)
                                    {
                                        m_module_reports.erase(it);
                                    }
                                    else if (msg.deviceID == it->deviceID)
                                    {
                                        m_module_reports.erase(it);
                                    }
                                }
                            }
                        }
                        if(!bChanged) {bChanged = (_count != m_module_reports.size());}
                    }
                    break;

                    case ads_msgs::ads_module_report_item::moduleStatus_Critical:
                        {
                            if(!bChanged) {_count = m_module_reports.size();}
                            szCritical = szOut;
                            m_module_reports.insert(msg);
                            if(!bChanged) {bChanged = (_count != m_module_reports.size());}
                        }
                    break;

                    case ads_msgs::ads_module_report_item::moduleStatus_Warning:
                        {
                            if(!bChanged) {_count = m_module_reports.size();}
                            szWarning = szOut;
                            m_module_reports.insert(msg);
                            if(!bChanged) {bChanged = (_count != m_module_reports.size());}
                        }
                    break;
                    }
                }
                
                if(!bChanged) return;

                {
                    if (s_my_config.m_debug & 0x01)
                    {
                        std::cout << "m_module_reports:" << m_module_reports.size() << std::endl;
                    }
                    
                    ads_str_array faultArray;
                    if(m_module_reports.size() > 0)
                    {
                        for (auto it = m_module_reports.begin(); it != m_module_reports.end(); it++)
                        {
                            std::ostringstream result;
                            result << (int)it->moduleID
                                << "-" << (int)it->moduleStatus
                                << "-" << (int)it->moduleSubtype;
                            if (0 != it->deviceID)
                            {
                                result  << "-" << (int)it->deviceID;
                            }

                            faultArray.items.push_back(result.str());

                            if (s_my_config.m_debug & 0x01)
                            {
                                std::cout << "\t" << result.str() << std::endl;
                            }
                        }
                    }
                    else
                    {
                        faultArray.items.push_back("0-0-0");
                    }
                    m_Pubs["/ads_fault_list"].publish(faultArray);
                }
            }

            CEventReport report;
            report.m_ItemName = Item_module_state_changed;

            if (!szCritical.empty())
            {
                report.m_value = szCritical;
                if(s_my_config.m_charge_fault)
                {
                    int value[2] = {0, 0};
                    QueryFaultAmount(IFaultProvider::StopAd, value[0], value[1]);
                    if(value[0] > 0)
                    {
                        ads_msgs::ads_ad_command msg_cmd;
                        msg_cmd.action = ads_msgs::ads_ad_command::action_Stop;
                        m_Pubs["/ads_ad_command"].publish(msg_cmd);
                        ros::spinOnce();
                    }
                }
            }
            else if (!szWarning.empty())
            {
                report.m_value = szWarning;
            }

            if (report.m_value.empty())
            {
                report.m_value = std::string("0-0-0");
            }

            //std::cout << __FUNCTION__ << std::endl;
            
            m_eventSignal(report);
        }

        void OnMsg_ads_ad_report(const ads_msgs::ads_ad_report& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            m_msg_ads_ad_report = msg;
            switch(m_msg_ads_ad_report.carState)
            {
            case ads_msgs::ads_ad_report::carState_Stop:
            {
                static const std::map<std::string, boost::any> property_value = 
                {
                    {IApi4HMI::Item_spout_water,    0},
                    {IApi4HMI::Item_brush_status,   0},
                    {IApi4HMI::Item_width_light,    0},
                    {IApi4HMI::Item_low_beam_light, 0},
                    {IApi4HMI::Item_high_beam_light,0},
                    {IApi4HMI::Item_light,          0},
                    {IApi4HMI::Item_reverse_light,  0},
                    {IApi4HMI::Item_brake_light,    0},
                    {IApi4HMI::Item_left_light,     0},
                    {IApi4HMI::Item_right_light,    0},
                    {IApi4HMI::Item_suction_status, 0},
                };
                SetProperty(property_value);

                ads_msgs::ads_ad_command msg_cmd;
                msg_cmd.action = ads_msgs::ads_ad_command::action_Stop;
                m_Pubs["/ads_ad_command"].publish(msg_cmd);
            }
            break;

            default:
            case ads_msgs::ads_ad_report::carState_Idle:
            case ads_msgs::ads_ad_report::carState_Moving:
            case ads_msgs::ads_ad_report::carState_MoveDone:
            case ads_msgs::ads_ad_report::carState_Sweeping:
            case ads_msgs::ads_ad_report::carState_SweepDone:
            case ads_msgs::ads_ad_report::carState_Manual:
            case ads_msgs::ads_ad_report::carState_Busy:
            break;
            }
        }

        void OnMsg_report_ThrottleInfo(const sweep_msgs::ThrottleReport & msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            sweep_msgs::ThrottleReport m_ThrottleReport = msg;
        }

        void OnMsg_report_BrakeInfo(const sweep_msgs::BrakeReport& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            sweep_msgs::BrakeReport m_BrakeReport = msg;
        }
        void OnMsg_report_SteerInfo(const sweep_msgs::SteeringReport& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            sweep_msgs::SteeringReport m_SteeringReport = msg;
        }
        void OnMsg_report_ModeInfo(const sweep_msgs::SystemMode& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            sweep_msgs::SystemMode m_SystemMode = msg;
        }
        void OnMsg_report_LightInfo(const sweep_msgs::Light& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            sweep_msgs::Light m_Light = msg;
        }
        void OnMsg_report_BmsInfo(const sweep_msgs::FuelLevelReport& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            m_FuelLevelReport = msg;
        }

        void OnMsg_ads_chasiss_status(const ads_msgs::Status& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            m_chasiss_status = msg;
        }

        void OnMsg_ads_ins_data(const ads_msgs::ads_ins_data& msg)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            m_ads_ins_data = msg;
        }

        void OnMsg_ads_planning_data(const ads_msgs::ads_PlanningData4Debug &data)
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            m_planningDataDebug = data;
        }

        bool Init(void)
        {
            if (m_bRunning) return false;
            m_bRunning = true;

            m_hostApi->getNodeHandle(this->s_nodeHandles);

            m_lpFaultDataBase.reset(new CFaultDataBase(s_my_config.m_fault_code_file));
            this->m_module_reports.clear();
            if(g_bEnableFaultHandle)
            {
                m_lpFaultDataBase->GetDefaultFaultList(this->m_module_reports);
            }

            std::string topic = "/ads_ad_command";
            m_Pubs[topic] = s_nodeHandles[0]->advertise<ads_msgs::ads_ad_command>(topic, 1);

            topic = "/ads_site_data_path";
            m_Pubs[topic] = s_nodeHandles[0]->advertise<ads_msgs::ads_site_data_path>(topic, 1);

            topic = "/ads_fault_list";
            m_Pubs[topic] = s_nodeHandles[0]->advertise<ads_msgs::ads_str_array>(topic, 1);

            topic = "/ads_sweeper_control_i";
            m_Pubs[topic] = s_nodeHandles[0]->advertise<ads_msgs::ads_int2int_data>(topic, 1);

            //ros::message_traits::definition<ads_msgs::ads_ad_command>();

            topic = "/ads_module_report";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 100, &CApi4HMI::OnMsg_ads_module_report, this);

            topic = "/ads_ad_report";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_ads_ad_report, this);

            if (nullptr != getenv("ADS_CHASISS_VENDER_NAME"))
            {
                std::string vender(getenv("ADS_CHASISS_VENDER_NAME"));
                if(vender == "haide")
                {
                    
                }
            }

            topic = "/report/ThrottleInfo";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_report_ThrottleInfo, this);

            topic = "/report/BrakeInfo";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_report_BrakeInfo, this);

            topic = "/report/SteerInfo";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_report_SteerInfo, this);

            topic = "/report/ModeInfo";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_report_ModeInfo, this);

            topic = "/report/LightInfo";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_report_LightInfo, this);

            topic = "/report/BmsInfo";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_report_BmsInfo, this);

            topic = "/ads_string_command";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 10, &CApi4HMI::OnMsg_StringCallback, this);

            topic = "/status";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_ads_chasiss_status, this);

            topic = "/ads_imu_data";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_ads_ins_data, this);

            topic = "/planning_debug_data";
            m_Subs[topic] = s_nodeHandles[0]->subscribe(topic, 1, &CApi4HMI::OnMsg_ads_planning_data, this);

            return true;
        }

        bool DeInit(void)
        {
            if (!m_bRunning) return false;
            m_bRunning = false;

            m_lpFaultDataBase.reset();
            m_Subs.clear();
            m_Pubs.clear();

            return true;
        }

    public:
        CApi4HMI(const std::string& json_file)
        {
            s_my_config.m_fault_code_file = json_file;

            {
                std::ifstream is;  
                is.open (json_file, std::ios::binary);    
                if(is.is_open())
                {
                    Json::Reader reader;
                    Json::Value root;
                    if (reader.parse(is, root, false))
                    {
                        auto& node = root["ignore_fault_list"];
                        if(!node.isNull())
                        {
                            for(int i = 0; i < node.size(); i++)
                            {
                                s_my_config.m_ignore_fault_list.insert(node[i].asString());
                            }
                        }

                        node = root["charge_fault"];
                        if(!node.isNull())
                        {
                            s_my_config.m_charge_fault = node.asInt();
                        }

                        node = root["fault_code_file"];
                        if(!node.isNull())
                        {
                            s_my_config.m_fault_code_file = node.asString();
                        }

                        node = root["debug"];
                        if(!node.isNull())
                        {
                            s_my_config.m_debug = node.asInt();
                        }
                        
                    }
                }
            }
        }

        virtual std::shared_ptr<ISelfCheck> GetSelfCheck(ISelfCheck::e_type type) override
        {
            switch (type)
            {
            case ISelfCheck::System:
            case ISelfCheck::Sensor:
            case ISelfCheck::Algorithm:
            case ISelfCheck::Chassis:
            {
                if(m_SelfCheckPool.find(type) == m_SelfCheckPool.end())
                {
                    m_SelfCheckPool[type] = std::make_shared<CSelfCheckImp>(this, type);
                }
                return m_SelfCheckPool[type];
            }
            break;
            default:
                return nullptr;
            }
        }

        virtual bool GetFaultList(std::vector<CFaultInfo>& faults) override
        {
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            for (auto it = m_module_reports.begin(); it != m_module_reports.end(); it++)
            {
                if (it->moduleStatus != ads_msgs::ads_module_report_item::moduleStatus_OK)
                {
                    CFaultInfo fault;
                    this->m_lpFaultDataBase->FillCFaultInfo(*it, fault);
                    faults.push_back(fault);
                }
            }
            if (s_my_config.m_debug & 0x02)
            {
                std::cout << "Total faults=" << faults.size() << std::endl;
            }
            return true;
        }

        virtual std::map<std::string, boost::any> GetProperty(const std::vector<std::string>& items) override
        {
            //ads_msgs::ads_ad_report m_msg_ads_ad_report;

            std::map<std::string, boost::any> values;
            std::lock_guard<std::recursive_mutex> lock(m_mutex);
            for (auto it = items.begin(); it != items.end(); it++)
            {
                if (*it == IApi4HMI::Item_spout_water)
                {
                    //5 - 8	工作模式	0：手持抽吸 | 1：干式清扫 | 2：湿式清扫 | 3：专用功能 | 4 - 14：reserved | 15：NA
                    switch(m_chasiss_status.working_mode)
                    {
                    case 2:
                        values[*it] = 1;
                    break;
                    
                    default:
                        values[*it] = 0;
                    break;
                    }
                }
                else if (*it == IApi4HMI::Item_brush_status)
                {
                    //上装工作使能	00：禁止 | 01：使能 | 10：NA | 11：无动作
                    values[*it] = m_chasiss_status.sweeper_enable;
                }
                // else if (*it == IApi4HMI::Item_manual_brake)
                // {
                //     values[*it] = 1;
                // }
                else if (*it == IApi4HMI::Item_width_light)
                {
                    values[*it] = this->m_chasiss_status.width_light;
                }
                // else if (*it == IApi4HMI::Item_emergency_light)
                // {
                //     values[*it] = 0;
                // }
                else if (*it == IApi4HMI::Item_low_beam_light)
                {
                    values[*it] = this->m_chasiss_status.low_beam_light;
                }
                // else if (*it == IApi4HMI::Item_fault_list)
                // {
                //     values[*it] = 1;
                // }
                else if (*it == IApi4HMI::Item_high_beam_light)
                {
                    values[*it] = this->m_chasiss_status.high_beam_light;
                }
                else if (*it == IApi4HMI::Item_car_state)
                {
                    values[*it] = static_cast<ads_ad_report::_carState_type>(m_msg_ads_ad_report.carState);
                }
                // else if (*it == IApi4HMI::Item_water_level)
                // {
                //     values[*it] = 10;
                // }
                else if (*it == dbAds::IApi4HMI::Item_gear)
                {
                    values[*it] = this->m_chasiss_status.gear;
                }
                else if (*it == dbAds::IApi4HMI::Item_light)
                {
                    values[*it] = this->m_chasiss_status.light;
                }
                else if (*it == dbAds::IApi4HMI::Item_reverse_light)
                {
                    values[*it] = this->m_chasiss_status.reversing_light;
                }
                else if (*it == dbAds::IApi4HMI::Item_brake_light)
                {
                    values[*it] = this->m_chasiss_status.brake_light;
                }
                else if (*it == dbAds::IApi4HMI::Item_left_light)
                {
                    values[*it] = this->m_chasiss_status.left_turn_light;
                }
                else if (*it == dbAds::IApi4HMI::Item_right_light)
                {
                    values[*it] = this->m_chasiss_status.right_turn_light;
                }
                else if (*it == dbAds::IApi4HMI::Item_suction_status)
                {
                    values[*it] = this->m_chasiss_status.fan_hz > 0 ? 1 : 0;
                }
                else if (*it == dbAds::IApi4HMI::Item_velocity)
                {
                    values[*it] = this->m_chasiss_status.speed;
                }
                else if (*it == dbAds::IApi4HMI::Item_battery_remaining_capacity)
                {
                    values[*it] = this->m_chasiss_status.battery;
                } 
                else if (*it == dbAds::IApi4HMI::Item_total_time)
                {
                    values[*it] = this->m_chasiss_status.total_time;
                }
                else if (*it == dbAds::IApi4HMI::Item_total_mileage)
                {
                    values[*it] = this->m_chasiss_status.total_mileage;
                }
                else if (*it == IApi4HMI::Item_latitude)
                {   
                    values[*it] = m_ads_ins_data.lat;
                }
                else if (*it == IApi4HMI::Item_longitude)
                {
                    values[*it] = m_ads_ins_data.lon;
                }
                else if (*it == IApi4HMI::Item_yaw_angle)
                {
                    values[*it] = m_ads_ins_data.yaw;
                }
                else if (it->compare(dbAds::IApi4HMI::Item_planning_data_debug) == 0)
                {
                    values[*it] = this->m_planningDataDebug;
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

        //自动驾驶，清扫到指定位置。全0.0意味着循环
        virtual bool SweepTo(double x = 0.0, double y = 0.0, double z = 0.0) override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = ads_ad_command::action_SweepTo;
            msg.lat = x;
            msg.lon = y;

            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }

        //退出自动驾驶
        virtual bool Stop() override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = ads_ad_command::action_Stop;

            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }

        virtual boost::signals2::connection InstallEventCallback(
            std::function<int(CEventReport&)> callback, const std::vector<std::string>& cared_property)override
        {
            m_cared_property = cared_property;
            return m_eventSignal.connect(callback);
        }

        virtual bool StartWithHost(IHostApi4HMI* host) override
        {
            m_hostApi = host;
            return Init();
        }
        
        //自动驾驶，移动到指定位置，全0.0意味着循环
        virtual bool MoveTo(double x, double y, double z) override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = ads_ad_command::action_MoveTo;
            msg.lat = x;
            msg.lon = y;

            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }
  
        //出库
        virtual bool MoveTo(int path) override
        {
            return true;
        }

        //路边停车, 停车后应退出自动驾驶，所以会有Stop命令跟随
        virtual bool StopByRoadSide() override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = ads_ad_command::action_RoadSideStop;
            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }

        //暂停，保持自动驾驶状态， 可以被resume
        virtual bool Pause() override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = ads_ad_command::action_Pause;
            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }

        //继续自动驾驶
        virtual bool Resume() override
        {
            ads_msgs::ads_ad_command msg;
            msg.action = ads_ad_command::action_Resume;
            m_Pubs["/ads_ad_command"].publish(msg);
            return true;
        }

        //选择清扫场景
        virtual bool SetTask(const std::string& strSiteName, const std::string& strJobName) override
        {
            ads_msgs::ads_site_data_path msg;
            msg.szSiteName = strSiteName;
            msg.szJobName = strJobName;
            m_Pubs["/ads_site_data_path"].publish(msg);
            return true;
        }

        //停车入库
        virtual bool ParkTo(int path) override
        {
            return true;
        }

        virtual boost::any GetProperty(const std::string& property) override
        {
            std::vector<std::string> properties{property};
            auto values = GetProperty(properties);
            if(values.size() == 1)
            {
                return values.begin()->second;
            }
            else
            {
                std::cout << "Failed to get property for: " << property;
                return boost::any();
            }
        }

        template<typename T> T toValue(T, const boost::any& data)
        {
            if (data.type() == typeid (int))
            {
                return boost::any_cast<int>(data);
            }
            else if (data.type() == typeid (float))
            {
                return boost::any_cast<float>(data);
            }
            else if (data.type() == typeid (double))
            {
                return boost::any_cast<double>(data);
            }
            else if (data.type() == typeid (std::string))
            {
                return std::strtod(boost::any_cast<std::string>(data).c_str(), nullptr);
            }
            else if (data.type() == typeid (bool))
            {
                return boost::any_cast<bool>(data);
            }
            else
            {
                std::cout << "Unsuportd type:" << data.type().name() << std::endl;
            }
            return -1;
        }

        virtual bool SetProperty(const std::map<std::string, boost::any>& values) override
        {
            ads_int2int_data msg;
            ads_int2int_item item;

            static const std::map<std::string, int> map_ = 
            {
                {IApi4HMI::Item_spout_water,    ads_sweeper_control_id::item_spout_water_run},
                {IApi4HMI::Item_brush_status,   ads_sweeper_control_id::item_brush_run},
                {IApi4HMI::Item_width_light,    ads_sweeper_control_id::item_width_light},
                {IApi4HMI::Item_low_beam_light, ads_sweeper_control_id::item_low_beam_light},
                {IApi4HMI::Item_high_beam_light,ads_sweeper_control_id::item_high_beam_light},
                {IApi4HMI::Item_light,          ads_sweeper_control_id::item_light},
                {IApi4HMI::Item_reverse_light,  ads_sweeper_control_id::item_reversing_light},
                {IApi4HMI::Item_brake_light,    ads_sweeper_control_id::item_brake_light},
                {IApi4HMI::Item_left_light,     ads_sweeper_control_id::item_left_turn_light},
                {IApi4HMI::Item_right_light,    ads_sweeper_control_id::item_right_turn_light},
                {IApi4HMI::Item_suction_status, ads_sweeper_control_id::item_suction_run},

                // {IApi4HMI::Item_emergency_light, 0},
                // {IApi4HMI::Item_fault_list, 0},
                // {IApi4HMI::Item_car_state, 0},
                // {IApi4HMI::Item_water_level, 0},
                // {IApi4HMI::Item_gear, 0},
            };                
                            
            for (auto it = values.begin(); it != values.end(); it++)
            {
                auto key_ = map_.find(it->first);
                if(key_ != map_.end())
                {
                    item.key = key_->second;
                }
                else
                {
                    std::cout << "Unknown Property:" << it->first << std::endl;
                    continue;
                }
                item.value = toValue(item.value, it->second);
                msg.items.push_back(item);
            }

            if(msg.items.size() > 0)
            {
                std::string topic = "/ads_sweeper_control_i";
                m_Pubs[topic].publish(msg);
            }

            return true;
        }

        virtual bool SetProperty(const std::string& property, const boost::any& value) override
        {
            std::map<std::string, boost::any> data = 
            {
                {property, value},
            };
            return SetProperty(data);
        }

        virtual std::vector<SiteJobItem> GetSiteJobsInfo(void)
        {
            std::vector<SiteJobItem> siteJobs;
            ros::ServiceClient client = s_nodeHandles[0]->serviceClient<srv_get_site_job_info>("srv_get_site_job_info");
            srv_get_site_job_info srv;
            srv.request.type = "all";
            if (client.call(srv))
            {
                for(auto& it : srv.response.result)
                {
                    SiteJobItem item;
                    item.m_ImgFilePath = it.m_ImgFilePath;
                    item.m_JobName = it.m_JobName;
                    item.m_RoadSideFilePath = it.m_RoadSideFilePath;
                    item.m_SiteName = it.m_SiteName;
                    siteJobs.push_back(item);
                }
            } 
            else 
            {
                ROS_ERROR("Failed to call service srv_get_site_job_info");
            }
            return siteJobs;
        }

        virtual bool AdIsReady(int) override
        {
            if(g_bEnableFaultHandle)
            {
                int value[2] = {0, 0};
                QueryFaultAmount(IFaultProvider::DisableAd, value[0], value[1]);
                //std::cout << __FUNCTION__ << ":" <<  value[0] << "," <<  value[1] << std::endl;
                return (value[0] == 0);
            }
            else
            {
                return true;
            }
        }

        //是否有critical fault发生，应该退出自动驾驶
        virtual bool AdNeedQuit(void) override
        {
            //Temp solution
            return false;

            if(m_chasiss_status.clear_auto_driving)
            {
                // ads_msgs::ads_ad_command msg_cmd;
                // msg_cmd.action = ads_msgs::ads_ad_command::action_Stop;
                // m_Pubs["/ads_ad_command"].publish(msg_cmd);
                std::cout << "clear_auto_driving=1(人工终止自动驾驶!)" << std::endl;

                return true;
            }

            if(!s_my_config.m_charge_fault)            
            {
                //改动：控制模块将控制车是否行进，直接返回false
                return false;
            }

            if(g_bEnableFaultHandle)
            {
                int value[2] = {0, 0};
                QueryFaultAmount(IFaultProvider::StopAd, value[0], value[1]);
                //std::cout << __FUNCTION__ << ":" <<  value[0] << "," <<  value[1] << std::endl;
                return (value[0] != 0);
            }
            else
            {
                return false;
            }
        }   

        virtual bool QueryFaultAmount(IFaultProvider::e_type type, int& iCritical, int& iWarning) override
        {
            iCritical = iWarning = 0;

            decltype(m_module_reports) data;
            //std::set<ads_msgs::ads_module_report_item, CModule_report_item_Comp> data;
            {
                std::lock_guard<std::recursive_mutex> lock(m_mutex);
                data = m_module_reports;
            }

            for(auto& item : data)
            {
                CFaultItemCategoryInfo* lp =  m_lpFaultDataBase->GetItemCategory(item.moduleID, item.moduleStatus, item.moduleSubtype);
                if(lp == nullptr) continue;

                switch (type)
                {
                case Chassis:
                case System:
                case Sensor:
                case Algorithm:
                    if(lp->m_category_post == type) 
                    {
                        switch(item.moduleStatus)
                        {
                        default:
                            break;
                        case ads_msgs::ads_module_report_item::moduleStatus_Critical:
                            iCritical++;
                            break;
                        case ads_msgs::ads_module_report_item::moduleStatus_Warning:
                            iWarning++;
                            break;
                        }
                    }
                break;

                case DisableAd:
                    if(lp->m_category_ad_init) 
                    {
                        switch(item.moduleStatus)
                        {
                        default:
                            break;
                        case ads_msgs::ads_module_report_item::moduleStatus_Critical:
                            iCritical++;
                            break;
                        case ads_msgs::ads_module_report_item::moduleStatus_Warning:
                            iWarning++;
                            break;
                        }
                    }
                break;

                case StopAd:
                    if(lp->m_category_ad) 
                    {
                        switch(item.moduleStatus)
                        {
                        default:
                            break;
                        case ads_msgs::ads_module_report_item::moduleStatus_Critical:
                            iCritical++;
                            break;
                        case ads_msgs::ads_module_report_item::moduleStatus_Warning:
                            iWarning++;
                            break;
                        }
                    }
                break;
                
                default:
                break;
                }
            }
            //std::cout << "iCritical=" << iCritical << ", iWarning" <<iWarning << std::endl;
            return true;
        }
    };
}

namespace dbAds
{
    std::ostream& operator <<(std::ostream&out, IApi4HMI::CFaultInfo& fault)
    {
        //return out;
        return out << "fault.szCode:" << fault.szCode << std::endl
            << "fault.szName:" << fault.szName << std::endl
            << "fault.szCondition:" << fault.szCondition << std::endl
            << "fault.szHandle:" << fault.szHandle << std::endl
            << "fault.szAction:" << fault.szAction << std::endl
            << "fault.szCategory" << fault.szCategory << std::endl;
    }

    std::ostream& operator <<(std::ostream&out, IApi4HMI::CEventReport& report)
    {
        //return out;
        if (report.m_value.type() == typeid (int))
        {
            auto value = boost::any_cast<int>(report.m_value);
            return std::cout << "report[\"" << report.m_ItemName << "\"]="
                << boost::any_cast<int>(report.m_value);
        }
        else if (report.m_value.type() == typeid (float))
        {
            auto value = boost::any_cast<float>(report.m_value);
            return std::cout << "report[\"" << report.m_ItemName << "\"]="
                << boost::any_cast<float>(report.m_value);
        }
        else if (report.m_value.type() == typeid (double))
        {
            auto value = boost::any_cast<double>(report.m_value);
            return std::cout << "report[\"" << report.m_ItemName << "\"]="
                << boost::any_cast<double>(report.m_value);
        }
        else if (report.m_value.type() == typeid (std::string))
        {
            auto value = boost::any_cast<std::string>(report.m_value);
            return std::cout << "report[\"" << report.m_ItemName << "\"]=\""
                << boost::any_cast<std::string>(report.m_value)
                << "\"";
        }
        else if (report.m_value.type() == typeid (ads_msgs::ads_ad_report::_carState_type))
        {
            static std::map<ads_msgs::ads_ad_report::_carState_type, std::string> names = 
            {
#define carStateItem(A) {ads_msgs::ads_ad_report::A, #A}
                carStateItem(carState_Idle),
                carStateItem(carState_Moving),
                carStateItem(carState_MoveDone),
                carStateItem(carState_Sweeping),
                carStateItem(carState_SweepDone),
                carStateItem(carState_Stop),
                carStateItem(carState_Manual),
                carStateItem(carState_Busy),
#undef carStateItem                
            };
            auto value = boost::any_cast<ads_msgs::ads_ad_report::_carState_type>(report.m_value);

            return std::cout << "report[\"" << report.m_ItemName << "\"]=\""
                << names[value] << "\"";
        }
        else
        {
            std::cout << report.m_ItemName << " with unsuportd type:" << report.m_value.type().name();
        }
    }

#define DECLARE_SWEEPER_CONTROL_DATA(name)   const std::string IApi4HMI::Item_##name = "data_"#name;
        #include "ISweeperControlData.h"
#undef DECLARE_SWEEPER_CONTROL_DATA

    const std::string config_filename = "dbAdsApi4HMI.json";

    IApi4HMI* getApi4HMI()
    {
        static CApi4HMI api(config_filename);
        return &api;
    }
}
