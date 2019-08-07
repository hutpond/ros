
#pragma once

#include <string>
#include <map>
#include <vector>
#include <functional> 
#include "boost/any.hpp" 

namespace dbAds
{
    class ISelfCheck
    {
    public:
        enum e_type
        {
            System,
            Sensor,
            Algorithm,
        };

        enum e_result
        {
            Pass,
            Failure,
            Unknown,
        };

        virtual ~ISelfCheck(){}
        virtual bool Start() = 0;
        virtual bool Stop() = 0;
        virtual int GetProgress() = 0;
        virtual e_result GetResult() = 0;
    };

    class IApi4HMI
    {
    public:
#define DECLARE_STATE_ITEM(name)   static const std::string Item_##name
        DECLARE_STATE_ITEM(shou_sha);
        DECLARE_STATE_ITEM(shikuo_light);
        DECLARE_STATE_ITEM(jinguang_light);
        DECLARE_STATE_ITEM(yuanguang_light);
        DECLARE_STATE_ITEM(yingji_light);
        DECLARE_STATE_ITEM(checliang_fault);
        DECLARE_STATE_ITEM(saopan);
        DECLARE_STATE_ITEM(penshui);
#undef DECLARE_STATE_ITEM

        class CFault
        {
        public:
            std::string szCode;
            std::string szDescription;
        };

        class CEventReport
        {
        public:
            std::string szCode;
        };

        virtual ISelfCheck* CreateCheck(ISelfCheck::e_type type) = 0;
        virtual bool GetFaultList(std::vector<CFault>& faults) = 0;

        virtual std::map<std::string, boost::any> GetValue(const std::vector<std::string>& list) = 0;

        virtual void ConnectPointCloud(std::string szName, bool b) = 0;

        virtual bool UpdateMapInfo(std::string strMapPath) = 0;
        virtual bool StartAutoDrive() = 0;
        virtual bool StopAutoDrive() = 0;

        virtual bool InstallNotify(std::function<int(CEventReport&)>) = 0;
    };

    IApi4HMI* getApi4HMI();
}