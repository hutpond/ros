#ifndef QCOSTVALUEQWIDGET_H
#define QCOSTVALUEQWIDGET_H

#include <QWidget>
#include "QPlanningCostWidget.h"

class QSlider;
class QPushButton;
class QLabel;

class QCostValueWidget : public QWidget
{
  Q_OBJECT

public:
  explicit QCostValueWidget(QWidget * = Q_NULLPTR);
  void updateSliderValue();
  static void getCostValue(double []);
  static void setOriginCostValue(double []);

protected slots:
  void onValueChanged(int);
  void onBtnReset();

signals:
  void costValueChanged();

private:
  QSlider *m_pSlider[QPlanningCostWidget::Count];
  QLabel *m_pLblValue[QPlanningCostWidget::Count];
  QPushButton *m_pBtnReset;
  QLabel *m_pLblSum;
  static int s_nValue[QPlanningCostWidget::Count];
  static int s_nOriginValue[QPlanningCostWidget::Count];
  static bool s_bFlagInit;
  static QCostValueWidget *s_pInstance;
};

#endif // QCOSTVALUEQWIDGET_H
