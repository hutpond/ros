#ifndef QITEMVALUEBASE_H
#define QITEMVALUEBASE_H

#include <QWidget>
#include <GlobalDefine.h>

class QItemValueBase : public QWidget
{
  Q_OBJECT
public:
  explicit QItemValueBase(QWidget *parent = nullptr);

  virtual void setPoint(const Point &) = 0;
  virtual Point getPoint(bool &) = 0;

signals:
  void saveData();
};

#endif // QITEMVALUEBASE_H
