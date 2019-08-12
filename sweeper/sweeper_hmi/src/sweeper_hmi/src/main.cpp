#include <iostream>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QDir>
#include "ros/ros.h"
#include "QDataManager.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SWEEPER_HMI_ROS");
  //QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
  QGuiApplication a(argc, argv);

  qmlRegisterType<QMsgInfo>("Sweeper.MsgInfo", 1, 0, "MsgInfo");
  qmlRegisterType<QDataManager>("Sweeper.DataManager", 1, 0, "DataManager");

  QQmlApplicationEngine engine;

  const QUrl url(QStringLiteral("qrc:/main.qml"));
  QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                   &a, [url](QObject *obj, const QUrl &objUrl) {
      if (!obj && url == objUrl)
          QCoreApplication::exit(-1);
  }, Qt::QueuedConnection);
  engine.load(url);

  return a.exec();
}
