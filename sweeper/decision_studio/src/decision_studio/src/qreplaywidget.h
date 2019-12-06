#ifndef QREPLAYWIDGET_H
#define QREPLAYWIDGET_H

#include <QWidget>

class QSlider;
class QPushButton;

class QReplayWidget : public QWidget
{
  Q_OBJECT

public:
  enum
  {
    BtnPlayPause,
    BtnPrevious,
    BtnNext,
    BtnPlayVelocity,
    BtnCount
  };

public:
  explicit QReplayWidget(QWidget *parent = nullptr);
  void setSliderSize(int);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

  void setPlayState(bool);
  void setPlayVelocity(int);

signals:
  void clicked(int, int);

protected slots:
  void onBtnClicked();
  void onSliderValueChanged(int);

private:
  QSlider *m_pSliderPlay;
  QPushButton *m_pBtnPlay[BtnCount];

  bool m_bFlagPlay;
  int m_nVelocity;
};

#endif // QREPLAYWIDGET_H
