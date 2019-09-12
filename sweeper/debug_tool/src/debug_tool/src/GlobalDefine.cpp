/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: GlobalDefine.cpp
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 定义全局变量与函数
********************************************************/
#include <sys/time.h>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <QStatusBar>
#include "GlobalDefine.h"
#include "QDebugToolMainWnd.h"

// 主显示字体
const QFont G_TEXT_FONT("Times", 11);
const QFont G_TEXT_SMALL_FONT("Times", 9);

QRectF g_rectfSweeper = QRectF(0, 0, 0, 0);

const double PI = 3.14159265;


/*******************************************************
 * @brief 状态栏显示信息
 * @param msg: 信息字符串

 * @return
********************************************************/
void setStatusMessage(const char *msg, int timeout)
{
  if (QDebugToolMainWnd::s_pStatusBar != NULL) {
    QDebugToolMainWnd::s_pStatusBar->showMessage(msg, timeout * 1000);
  }
}

/*******************************************************
 * @brief 创建本次运行时保存数据文件的路径
 * @param name: 子文件夹名称

 * @return: 创建的路径变量
********************************************************/
fs::path createPathInCurrent(const std::string &name)
{
  char time_str[64] = {0};
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);

  sprintf(time_str, "%04d%02d%02d_%02d%02d%02d",
          utcTime->tm_year + 1900,
          utcTime->tm_mon + 1,
          utcTime->tm_mday,
          utcTime->tm_hour,
          utcTime->tm_min,
          utcTime->tm_sec
          );
  std::string strPathName = time_str;

  fs::path path = fs::current_path();
  path /= name;
  fs::remove_all(path);
  path /= strPathName;
  if (!fs::exists(path)) {
    fs::create_directories(path);
  }
  return path;
}

/*******************************************************
 * @brief 创建根据时间生成的文件名
 * @param path: 文件夹变量
 * @param ext: 文件扩展名

 * @return: 文件名
********************************************************/
std::string createFileName(const fs::path &path, const std::string &ext)
{
  namespace fs = boost::filesystem;
  fs::path pathFile(path);

  char time_str[64] = {0};
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);

  struct timeval tv;
  gettimeofday(&tv, NULL);

  sprintf(time_str, "%04d%02d%02d_%02d%02d%02d_%3d",
          utcTime->tm_year + 1900,
          utcTime->tm_mon + 1,
          utcTime->tm_mday,
          utcTime->tm_hour,
          utcTime->tm_min,
          utcTime->tm_sec,
          static_cast<int>((tv.tv_usec / 1000) % 1000)
          );

  std::string strFileName = time_str;
  if (!ext.empty()) {
    strFileName += ext;
  }
  pathFile /= strFileName;
  strFileName = pathFile.string();

  return strFileName;
}
