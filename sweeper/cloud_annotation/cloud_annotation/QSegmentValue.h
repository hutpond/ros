#ifndef QSEGMENTVALUE_H
#define QSEGMENTVALUE_H

#include "QItemValueBase.h"

namespace Ui {
class QSegmentValue;
}

class QSegmentValue : public QItemValueBase
{
  Q_OBJECT

public:
  explicit QSegmentValue(QWidget *parent = nullptr);
  ~QSegmentValue();
  virtual void setPoint(const Point &) final;
  virtual Point getPoint(bool &) final;

private:
  Ui::QSegmentValue *ui;
};

#endif // QSEGMENTVALUE_H
