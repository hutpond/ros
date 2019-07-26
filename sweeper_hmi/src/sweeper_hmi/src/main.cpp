#include <iostream>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QDir>
#include "ros/ros.h"
#include "QSelfChecking.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SWEEPER_HMI_ROS");
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QGuiApplication a(argc, argv);

  QQmlApplicationEngine engine;
  qmlRegisterType<QSelfChecking>("Sweeper", 1, 0, "SelfChecking");

  engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
  if (engine.rootObjects().isEmpty())
    return -1;

  return a.exec();
}
