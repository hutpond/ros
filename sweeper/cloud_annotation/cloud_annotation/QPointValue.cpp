#include "QPointValue.h"
#include "ui_QPointValue.h"

QPointValue::QPointValue(QWidget *parent) :
  QItemValueBase(parent),
  ui(new Ui::QPointValue)
{
  ui->setupUi(this);

  connect(ui->pushButton, SIGNAL(clicked()), this, SIGNAL(saveData()));
}

QPointValue::~QPointValue()
{
  delete ui;
}

void QPointValue::setPoint(const Point &point)
{
  QString text = QString("%1").arg(point.x, 0, 'f', 3);
  ui->lineEdit_east->setText(text);
  text = QString("%1").arg(point.y, 0, 'f', 3);
  ui->lineEdit_north->setText(text);
  text = QString("%1").arg(point.z, 0, 'f', 3);
  ui->lineEdit_up->setText(text);
}

Point QPointValue::getPoint(bool &ok)
{
  Point point;
  bool ok_x, ok_y, ok_z;
  point.x = ui->lineEdit_east->text().toDouble(&ok_x);
  point.y = ui->lineEdit_north->text().toDouble(&ok_y);
  point.z = ui->lineEdit_up->text().toDouble(&ok_z);
  ok = (ok_x && ok_y && ok_z);
  return point;
}
