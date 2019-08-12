#pragma once

#include <string>
#include <map>
#include <vector>
#include <functional> 
#include "boost/any.hpp" 
#include "boost/signals2.hpp"
#include "ros/node_handle.h"
#include <sensor_msgs/PointCloud2.h>

namespace dbAds
{
class ISelfCheck
{
public:
  enum e_type
  {
    Vehicle,
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
    std::string szName;
    std::string szCondition;
    std::string szHandle;
    std::string szAction;
  };

  friend inline std::ostream& operator <<(std::ostream&out, CFault& fault)
  {
    return out << "fault.szCode:" << fault.szCode << std::endl
               << "fault.szName:" << fault.szName << std::endl
               << "fault.szCondition:" << fault.szCondition << std::endl
               << "fault.szHandle:" << fault.szHandle << std::endl
               << "fault.szAction:" << fault.szAction << std::endl
               << std::endl;
  }

  class CEventReport
  {
  public:
    std::string szCode;
  };

  virtual ~IApi4HMI(){}

  //A static pointer, do not delete.
  virtual ISelfCheck* GetSelfCheck(ISelfCheck::e_type type) = 0;
  virtual bool GetFaultList(std::vector<CFault>& faults) = 0;

  virtual std::map<std::string, boost::any> GetProperty(const std::vector<std::string>& items) = 0;

  virtual boost::signals2::connection ConnectPointCloud(const std::string& szName, std::function<void(const sensor_msgs::PointCloud2&)> callback) = 0;

  virtual bool UpdateMapInfo(const std::string& strMapPath) = 0;

  virtual bool StartAutoDrive() = 0;
  virtual bool StopAutoDrive() = 0;

  virtual boost::signals2::connection InstallEventCallback(std::function<int(CEventReport&)> callback) = 0;

  //When szFaultDataBaseFile.empty(), "./fault_code.xls" will be used.
  virtual bool StartWithHost(IHostApi4HMI* host, const std::string& szFaultDataBaseFile, int iDebug = 0) = 0;
};

IApi4HMI* getApi4HMI();
}
