#ifndef QPOINTVALUE_H
#define QPOINTVALUE_H

#include "QItemValueBase.h"

namespace Ui {
class QPointValue;
}

class QPointValue : public QItemValueBase
{
  Q_OBJECT

public:
  explicit QPointValue(QWidget *parent = nullptr);
  ~QPointValue();
  virtual void setPoint(const Point &) final;
  virtual Point getPoint(bool &) final;

private:
  Ui::QPointValue *ui;
};

#endif // QPOINTVALUE_H
