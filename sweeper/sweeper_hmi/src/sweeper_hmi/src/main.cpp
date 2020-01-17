#include <unistd.h>
#include <execinfo.h>
#include <signal.h>
#include <iostream>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QDir>
#include <QDateTime>
#include "ros/ros.h"
#include "QDataManager.h"

void trace(int);
static QObject * dataManagerProvider(QQmlEngine *, QJSEngine *);

int main(int argc, char **argv)
{
  signal(SIGSEGV, trace);
  signal(SIGINT, trace);
  signal(SIGQUIT, trace);

  ros::init(argc, argv, "SWEEPER_HMI_ROS");
  //QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
  QGuiApplication a(argc, argv);

  qmlRegisterType<QMsgInfo>("Sweeper.MsgInfo", 1, 0, "MsgInfo");
  qmlRegisterSingletonType<QDataManager>("Sweeper.DataManager", 1, 0, "DataManager", dataManagerProvider);

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

QObject * dataManagerProvider(QQmlEngine *engine, QJSEngine *scriptEngine)
{
    Q_UNUSED(engine)
    Q_UNUSED(scriptEngine)

    QDataManager *manager = new QDataManager();
    return manager;
}

void trace(int signo)
{
  constexpr int SIZE = 100;
  void *buffer[100];
  char **strings;

  int nptrs = backtrace(buffer, SIZE);
  strings = backtrace_symbols(buffer, nptrs);
  if (strings == nullptr) {
    perror("backtrace_symbols");
    exit(EXIT_FAILURE);
  }

  QString fileName = QString::fromStdString(getenv("HOME"));
  if (!fileName.endsWith('/')) {
    fileName.push_back('/');
  }
  fileName.append(".sweeper_hmi/");
  QDir dir;
  if (!dir.exists(fileName)) {
    dir.mkdir(fileName);
  }

  QDateTime dateTime = QDateTime::currentDateTime();
  fileName += QString("log_%1_%2%3%4_%5%6%7.dump").
      arg(signo).arg(dateTime.date().year()).
      arg(dateTime.date().month()).
      arg(dateTime.date().day()).
      arg(dateTime.time().hour()).
      arg(dateTime.time().minute()).
      arg(dateTime.time().second());

  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    free(strings);
    return;
  }

  QTextStream out(&file);
  for (int i = 0; i < nptrs; ++i) {
    out << strings[i] << "\n";
  }
  file.close();

  free(strings);

  exit(0);
}
