
#include <QApplication>
#include "qcloudmainwnd.h"

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  QCloudMainWnd w;
  w.showMaximized();
  return a.exec();
}
