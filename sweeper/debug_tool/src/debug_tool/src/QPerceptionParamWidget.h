/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionParamWidget.h
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: Perception模块参数显示、设定类
********************************************************/
#ifndef Q_PERCEPTION_PARAM_WIDGET_H
#define Q_PERCEPTION_PARAM_WIDGET_H

#include <QWidget>

class QUltrasonicWidget;
class QLabel;
class QSlider;
class QGroupBox;
class QPushButton;
class QSlider;
struct PerceptionData;

class QPerceptionParamWidget : public QWidget
{
  Q_OBJECT

public:
  QPerceptionParamWidget(QWidget *parent);
  ~QPerceptionParamWidget();

  void setCipvData(int, float);
  void setRoadSide(float, float);
  void setPerceptionData(const PerceptionData *);
  void setShowType(int);
  void setFrameCount(int);
  void setFrameOffset(int);

signals:
  void replayState(bool);
signals:
  void replayFrameOffset(int);
  void displayData();

private slots:
  void onBtnClicked();
  void onSliderValueChanged(int);

private:
  QUltrasonicWidget *m_pWdgUltrasonic;

  QLabel *m_pLblCipvType;
  QLabel *m_pLblCipvDistance;
  QLabel *m_pLblRoadSide;

  QGroupBox *m_pGBoxReplay;
  QPushButton *m_pBtnPause;
  QPushButton *m_pBtnResume;
  QPushButton *m_pBtnBack;
  QPushButton *m_pBtnNext;
  QPushButton *m_pBtnData;
  QSlider *m_pSliderPlay;

  int m_nSliderValue;
};

#endif // Q_PERCEPTION_PARAM_WIDGET_H
