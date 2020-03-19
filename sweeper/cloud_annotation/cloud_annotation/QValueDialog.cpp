#include "QValueDialog.h"

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>

QValueDialog::QValueDialog(int count, QWidget *parent)
  : QDialog(parent)
{
  QVBoxLayout *v_layout = new QVBoxLayout;

  for (int i = 0; i < count; ++i) {
    QLabel *title = new QLabel(this);
    title->setMinimumSize(QSize(100, 30));
    lbl_titles_.push_back(title);

    QLineEdit *value = new QLineEdit(this);
    value->setMinimumSize(QSize(300, 30));
    edit_value_.push_back(value);

    QHBoxLayout *h_layout = new QHBoxLayout;
    h_layout->addWidget(title, 1);
    h_layout->addWidget(value, 3);

    v_layout->addLayout(h_layout);
  }

  btn_ok_ = new QPushButton(this);
  btn_ok_->setText("OK");
  btn_cancel_ = new QPushButton(this);
  btn_cancel_->setText("Cancel");
  QHBoxLayout *h_layout = new QHBoxLayout;
  h_layout->addStretch();
  h_layout->addWidget(btn_ok_);
  h_layout->addStretch();
  h_layout->addWidget(btn_cancel_);
  h_layout->addStretch();
  v_layout->addLayout(h_layout);

  this->setLayout(v_layout);

  connect(btn_ok_, &QPushButton::clicked, this, &QDialog::accept);
  connect(btn_cancel_, &QPushButton::clicked, this, &QDialog::reject);
}

void QValueDialog::setTitle(const QStringList &titles)
{
  assert(titles.size() == lbl_titles_.size());
  const int size_title = titles.size();
  for (int i = 0; i < size_title; ++i) {
    lbl_titles_[i]->setText(titles[i]);
  }
}

double QValueDialog::value(int index, bool *ok)
{
  assert(index >= 0 && index < edit_value_.size());
  double value = edit_value_[index]->text().toDouble(ok);
  return value;
}
