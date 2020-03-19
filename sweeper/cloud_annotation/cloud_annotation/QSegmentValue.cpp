#include "QSegmentValue.h"
#include "ui_QSegmentValue.h"

QSegmentValue::QSegmentValue(QWidget *parent) :
  QItemValueBase(parent),
  ui(new Ui::QSegmentValue)
{
  ui->setupUi(this);
  connect(ui->pushButton, SIGNAL(clicked()), this, SIGNAL(saveData()));
}

QSegmentValue::~QSegmentValue()
{
  delete ui;
}

void QSegmentValue::setPoint(const Point &point)
{
  QString text = QString::number(static_cast<int>(point.x));
  ui->lineEdit->setText(text);
  text = text = QString::number(static_cast<int>(point.y));
  ui->lineEdit_2->setText(text);
}

Point QSegmentValue::getPoint(bool &ok)
{
  Point point;
  bool ok_x, ok_y;
  point.x = ui->lineEdit->text().toInt(&ok_x);
  point.y = ui->lineEdit_2->text().toInt(&ok_y);
  ok = (ok_x && ok_y);
  return point;
}
