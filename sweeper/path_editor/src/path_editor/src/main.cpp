/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: main.cpp
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 程序入口
********************************************************/
#include "QEditorMainWindow.h"
#include <QApplication>
#include <QScreen>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  QEditorMainWindow w;
#ifdef DEBUG
  QList<QScreen *> screens = QGuiApplication::screens();
  QRect rect = screens.at(0)->availableGeometry();
  QPoint pt = rect.center();
  rect.setSize(QSize(1024, 768));
  rect.moveCenter(pt);
  w.setGeometry(rect);
#else
  w.showMaximized();
#endif
  w.show();

  return a.exec();
}
