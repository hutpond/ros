#ifndef QCOSTVALUEDIALOG_H
#define QCOSTVALUEDIALOG_H

#include <QDialog>
#include "QPlanningCostWidget.h"

class QSlider;
class QPushButton;
class QLabel;

class QCostValueDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QCostValueDialog(QWidget * = Q_NULLPTR);

protected:

protected:
  void setCostValue(double []);
  void calcPerccent(double []);

protected:
  void showEvent(QShowEvent *);

protected slots:
  void onValueChanged(int);

signals:
  void costValue(double []);

private:
  QSlider *m_pSlider[QPlanningCostWidget::Count];
  QLabel *m_pLblValue[QPlanningCostWidget::Count];
  static int s_nValue[QPlanningCostWidget::Count];
};

#endif // QCOSTVALUEDIALOG_H
