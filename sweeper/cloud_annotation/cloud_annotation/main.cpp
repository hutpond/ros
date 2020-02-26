#include "qcloudmainwnd.h"

#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  QDesktopWidget *desktop = QApplication::desktop();
  QRect rect = desktop->rect();

  QCloudMainWnd w;
  w.setGeometry(rect);
  w.show();
  return a.exec();
}
