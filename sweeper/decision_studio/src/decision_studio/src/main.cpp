#include <QApplication>
#include <QDesktopWidget>
#include <ros/ros.h>

#include "qstudiowindow.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "decistion_studio_subscriber_node");
  QApplication a(argc, argv);

  QStudioWindow w;
  QRect rect = QApplication::desktop()->screenGeometry();
  QPoint pt = rect.center();
  rect.setSize(QSize(1024, 768));
  rect.moveCenter(pt);
  w.setGeometry(rect);

  w.show();

  return a.exec();
}
