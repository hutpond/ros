#pragma once

#include <string>
#include <map>
#include <vector>
#include <functional> 
#include <memory>

#include "boost/any.hpp" 
#include "boost/signals2.hpp"

#include "ros/node_handle.h"

#include "sensor_msgs/PointCloud2.h"

namespace dbAds
{
   struct SiteJobItem
    {
        //作业地图选择页面：作业地点
        std::string m_SiteName; 

        //作业地图选择页面：作业任务
        std::string m_JobName;

        //作业地图选择页面：用来显示的图片    
        std::string m_ImgFilePath;

        //自动驾驶页面：参考线经纬度，左、右车道线经纬度
        std::string m_RoadSideFilePath;
    };

    class ISelfCheck
    {
    public:
        enum e_type
        {
            Chassis,
            System,
            Sensor,
            Algorithm,
        };

         enum e_result
        {
            Pass,
            Failure,
            Processing,
            Unknown,
        };

        virtual ~ISelfCheck(){}
        virtual bool Start() = 0;
        virtual bool Stop() = 0;
        virtual int GetProgress() = 0;
        virtual e_result GetResult() = 0;
    };

    class IHostApi4HMI
    {
    public:
        virtual ~IHostApi4HMI(){}
        virtual void getNodeHandle(std::vector<ros::NodeHandle*>&) = 0;
    };

    class IApi4HMI
    {
    public:
#define DECLARE_SWEEPER_CONTROL_DATA(name)   static const std::string Item_##name;
        #include "ISweeperControlData.h"
#undef DECLARE_SWEEPER_CONTROL_DATA

        class CFaultInfo
        {
        public:
            std::string szCategory;
            std::string szCode;
            std::string szName;
            std::string szCondition;
            std::string szHandle;
            std::string szAction;
            friend std::ostream& operator <<(std::ostream&out, CFaultInfo& fault);
        };

        class CEventReport
        {
        public:
            std::string m_ItemName;
            boost::any m_value;
            friend std::ostream& operator <<(std::ostream&out, CEventReport& report);
        };

        virtual ~IApi4HMI(){}

        virtual std::shared_ptr<ISelfCheck> GetSelfCheck(ISelfCheck::e_type type) = 0;

        virtual bool GetFaultList(std::vector<CFaultInfo>& faults) = 0;

        virtual std::map<std::string, boost::any> GetProperty(const std::vector<std::string>& properties) = 0;
        virtual boost::any GetProperty(const std::string& property) = 0;

        virtual bool SetProperty(const std::map<std::string, boost::any>& propValues) = 0;
        virtual bool SetProperty(const std::string& property, const boost::any& value) = 0;

        virtual boost::signals2::connection ConnectPointCloud(const std::string& szName, std::function<void(const sensor_msgs::PointCloud2&)> callback) = 0;

        //自动驾驶，移动到指定位置，全0.0意味着循环
        virtual bool MoveTo(double x = 0.0, double y = 0.0, double z = 0.0) = 0;
  
        //自动驾驶，清扫到指定位置。全0.0意味着循环
        virtual bool SweepTo(double x = 0.0, double y = 0.0, double z = 0.0) = 0;

        //退出自动驾驶
        virtual bool Stop() = 0;

        //出库，保持自动驾驶状态
        virtual bool MoveTo(int path) = 0; 

        //路边停车, 停车后应退出自动驾驶，所以会有Stop命令跟随
        virtual bool StopByRoadSide() = 0;

        //暂停，保持自动驾驶状态， 可以被resume
        virtual bool Pause() = 0;

        //继续
        virtual bool Resume() = 0;

        //选择清扫场景
        virtual bool SetTask(const std::string& strSiteName, const std::string& strJobName) = 0;

        //停车入库
        virtual bool ParkTo(int path) = 0; 

        //是否能进入自动驾驶
        virtual bool AdIsReady(int) = 0;

        //是否有critical fault发生，应该退出自动驾驶
        virtual bool AdNeedQuit(void) = 0;

        virtual boost::signals2::connection InstallEventCallback(
            std::function<int(CEventReport&)> callback, const std::vector<std::string>& cared_property) = 0;

        virtual bool StartWithHost(IHostApi4HMI* host) = 0;

        virtual std::vector<SiteJobItem> GetSiteJobsInfo(void) = 0;
    };

    IApi4HMI* getApi4HMI();
}
