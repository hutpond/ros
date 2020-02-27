
#include <QApplication>
#include <QGLFormat>
#include "qcloudmainwnd.h"

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  if (!QGLFormat::hasOpenGL()) {
    qWarning( "This system has no OpenGL support. Exiting." );
    return -1;
  }
  QCloudMainWnd w;
  w.showMaximized();
  return a.exec();
}
