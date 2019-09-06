#ifndef QCOSTVALUEDIALOG_H
#define QCOSTVALUEDIALOG_H

#include <QDialog>
#include "QPlanningCostWidget.h"

class QLineEdit;
class QPushButton;

class QCostValueDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QCostValueDialog(QWidget * = Q_NULLPTR);
  void setCostValue(double []);
  void getCostValue(double []);

private:
  QLineEdit *m_pEditValue[QPlanningCostWidget::Count];
  QPushButton *m_pBtnOk;
  QPushButton *m_pBtnCancel;
};

#endif // QCOSTVALUEDIALOG_H
