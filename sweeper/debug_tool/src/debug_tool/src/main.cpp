#include <QtWidgets/QApplication>
#include <QDesktopWidget>
#include "ros/ros.h"
#include "QDebugToolMainWnd.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sweeper_debug_tool_node");
  QApplication a(argc, argv);

  QDebugToolMainWnd w;
  QRect rect = QApplication::desktop()->screenGeometry();
//#ifdef TEST
  QPoint pt = rect.center();
  rect.setSize(QSize(1024, 768));
  rect.moveCenter(pt);
//#else
//  w.showMaximized();
//#endif
  w.setGeometry(rect);
  w.show();

  int ret = a.exec();
  w.stopProcess();

  return ret;
}
