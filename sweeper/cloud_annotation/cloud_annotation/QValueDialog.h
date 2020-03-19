#ifndef QVALUEDIALOG_H
#define QVALUEDIALOG_H

#include <QDialog>

class QLabel;
class QLineEdit;

class QValueDialog : public QDialog
{
public:
  explicit QValueDialog(int, QWidget *);

  void setTitle(const QStringList &);
  double value(int, bool *);

private:
  QList<QLabel *> lbl_titles_;
  QList<QLineEdit *> edit_value_;
  QPushButton *btn_ok_;
  QPushButton *btn_cancel_;
};

#endif // QVALUEDIALOG_H
