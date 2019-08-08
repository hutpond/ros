#include "dbAdsApi4HMI.h"

using namespace dbAds;

namespace{
    class CSelfCheckSystem : public ISelfCheck
    {
    public:
        virtual bool Start() override
        {
            return true;
        }

        virtual bool Stop() override
        {
            return true;
        }

        virtual int GetProgress() override
        {
            return 0;
        }

        virtual e_result GetResult() override
        {
            return ISelfCheck::Unknown;
        }
    };

    class CSelfCheckAlgorithm : public ISelfCheck
    {
    public:
        virtual bool Start() override
        {
            return true;
        }

        virtual bool Stop() override
        {
            return true;
        }

        virtual int GetProgress() override
        {
            return 0;
        }

        virtual e_result GetResult() override
        {
            return ISelfCheck::Unknown;
        }
    };

    class CSelfCheckSensor : public ISelfCheck
    {
    public:
        virtual bool Start() override
        {
            return true;
        }

        virtual bool Stop() override
        {
            return true;
        }

        virtual int GetProgress() override
        {
            return 0;
        }

        virtual e_result GetResult() override
        {
            return ISelfCheck::Unknown;
        }
    };

    class CApi4HMI : public dbAds::IApi4HMI
    {
    public:
        CApi4HMI()
        {

        }

        virtual ISelfCheck* CreateCheck(ISelfCheck::e_type type) override
        {
            switch (type)
            {
            case ISelfCheck::e_type::System:
                return new CSelfCheckSystem();
                break;

            case ISelfCheck::e_type::Sensor:
                return new CSelfCheckSensor();
                break;

            case ISelfCheck::e_type::Algorithm:
                return new CSelfCheckAlgorithm();
                break;

            default:
                break;
            }
            return nullptr;
        }

        virtual bool GetFaultList(std::vector<CFault>& faults) override
        {
            return true;
        }

        virtual std::map<std::string, boost::any> GetValue(const std::vector<std::string>& list) override
        {

        }

        virtual void ConnectPointCloud(std::string szName, bool b) override
        {

        }

        virtual bool UpdateMapInfo(std::string strMapPath) override
        {

        }

        virtual bool StartAutoDrive() override
        {

        }

        virtual bool StopAutoDrive() override
        {

        }

        virtual bool InstallNotify(std::function<int(CEventReport&)>)override
        {

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

void test()
{
    dbAds::IApi4HMI* lp = dbAds::getApi4HMI();
    lp->CreateCheck(dbAds::ISelfCheck::e_type::Sensor);
    lp->GetValue({ dbAds::IApi4HMI::Item_penshui });
}
