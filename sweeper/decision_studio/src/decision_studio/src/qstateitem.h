#ifndef QSTATEITEM_H
#define QSTATEITEM_H

#include <QWidget>

class QStateItem : public QWidget
{
  Q_OBJECT
public:
  explicit QStateItem(const QString &, QWidget *parent = nullptr);

  void setStateOn(bool);

protected:
  virtual void resizeEvent(QResizeEvent *) final;
  virtual void paintEvent(QPaintEvent *) final;

private:
  QString m_strName;
  bool m_bFlagOn;

  QRect m_rectSvg;
  QRect m_rectName;
};

#endif // QSTATEITEM_H
