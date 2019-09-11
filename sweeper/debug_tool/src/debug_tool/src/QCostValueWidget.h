#ifndef QCOSTVALUEQWIDGET_H
#define QCOSTVALUEQWIDGET_H

#include <QDialog>
#include "QPlanningCostWidget.h"

class QSlider;
class QPushButton;
class QLabel;

class QCostValueWidget : public QWidget
{
  Q_OBJECT

public:
  explicit QCostValueWidget(QWidget * = Q_NULLPTR);
  static void getCostValue(double []);

protected slots:
  void onValueChanged(int);

signals:
  void costValueChanged();

private:
  QSlider *m_pSlider[QPlanningCostWidget::Count];
  QLabel *m_pLblValue[QPlanningCostWidget::Count];
  static int s_nValue[QPlanningCostWidget::Count];
};

#endif // QCOSTVALUEQWIDGET_H
