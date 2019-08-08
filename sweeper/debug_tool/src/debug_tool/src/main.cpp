#include <QtWidgets/QApplication>
#include <QDesktopWidget>
#ifndef WIN64
# include "ros/ros.h"
#endif
#include "QDebugToolMainWnd.h"

int main(int argc, char *argv[])
{
#ifndef WIN64
  ros::init(argc, argv, "sweeper_debug_tool_node");
#endif
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
