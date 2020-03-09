#ifndef QPOINTVALUE_H
#define QPOINTVALUE_H

#include <QWidget>
#include <GlobalDefine.h>

namespace Ui {
class QPointValue;
}

class QPointValue : public QWidget
{
  Q_OBJECT

public:
  explicit QPointValue(QWidget *parent = nullptr);
  ~QPointValue();
  void setPoint(const Point &);
  Point getPoint(bool &);

signals:
  void saveData();

private:
  Ui::QPointValue *ui;
};

#endif // QPOINTVALUE_H
