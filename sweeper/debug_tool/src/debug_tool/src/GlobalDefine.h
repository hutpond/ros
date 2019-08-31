/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: GlobalDefine.h
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 定义全局变量与函数
********************************************************/
#ifndef GLOBAL_DEFINE_H
#define GLOBAL_DEFINE_H

#include <boost/filesystem.hpp>
#include <QFont>

namespace fs = boost::filesystem;

extern const QFont G_TEXT_FONT;
extern const QFont G_TEXT_SMALL_FONT;

extern QRectF g_rectfSweeper;    // 扫地车位置
extern const double MAP_TO_ROAD_COEF;
extern const double VEH_HEAD;

extern const double PI;

void setStatusMessage(const char *, int = 0);

fs::path createPathInCurrent(const std::string &);
std::string createFileName(const fs::path &, const std::string &);

#endif // GLOBAL_DEFINE_H

