#include "QWaitingDialog.h"
#include "ui_QWaitingDialog.h"
#include <QMovie>

QWaitingDialog::QWaitingDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::QWaitingDialog)
{
  ui->setupUi(this);
  setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
  setAttribute(Qt::WA_TranslucentBackground);

  QMovie *movie = new QMovie(":/image/loading.gif");
  ui->label->setScaledContents(true);
  ui->label->setMovie(movie);
  movie->start();
}

QWaitingDialog::~QWaitingDialog()
{
  delete ui;
}
