/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: main.cpp
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 程序入口
********************************************************/
#include "QEditorMainWindow.h"
#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  QEditorMainWindow w;
#ifdef TEST
  QRect rect = QApplication::desktop()->screenGeometry();
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
