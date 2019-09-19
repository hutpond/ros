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
    Garbage,
    Save,
    Count
  };

public:
  explicit QEditToolsWidget(QWidget *parent = nullptr);

protected:
  void resizeEvent(QResizeEvent *) final;

signals:
  void selectTool(int, bool);

public slots:
  void onBtnClicked();

private:
  QPushButton *m_pBtnTool[Count];
  bool m_bFlagCheckable[Count];
  int m_nBtnIndex;
};

#endif // QEDITTOOLSWIDGET_H
