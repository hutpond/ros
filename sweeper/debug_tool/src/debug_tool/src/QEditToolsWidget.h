#ifndef QEDITTOOLSWIDGET_H
#define QEDITTOOLSWIDGET_H

#include <QWidget>

class QPushButton;

class QEditToolsWidget : public QWidget
{
  Q_OBJECT

public:
  enum {
    Move,
    Target,
    Count
  };

public:
  explicit QEditToolsWidget(QWidget *parent = nullptr);

protected:
  void resizeEvent(QResizeEvent *) final;

signals:
  void selectTool(int);

public slots:
  void onBtnClicked();

private:
  QPushButton *m_pBtnTool[Count];
  int m_nBtnIndex;
};

#endif // QEDITTOOLSWIDGET_H
