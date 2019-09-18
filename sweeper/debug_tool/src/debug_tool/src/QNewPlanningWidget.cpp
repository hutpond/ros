#include "QNewPlanningWidget.h"
#include "QPlanningParamWidget.h"
#include "GlobalDefine.h"
#include "QFullViewWidget.h"
#include "QReadDataManagerRos.h"
#include "QPlanningCostWidget.h"

QNewPlanningWidget::QNewPlanningWidget(QWidget *parent)
  : QBaseWidget(parent)
{
  this->setFont(G_TEXT_FONT);
  m_pWdgParam = new QPlanningParamWidget(this);
  m_pWdgParam->setShowType(LivePlay);
//  connect(m_pWdgParam, &QPlanningParamWidget::replayState,
//          this, &QBaseWidget::onReplayState);
//  connect(m_pWdgParam, &QPlanningParamWidget::replayFrameOffset,
//          this, &QBaseWidget::onSetFrameIndexReplay);
//  connect(m_pWdgParam, &QPlanningParamWidget::costValueChanged,
//          this, &QBaseWidget::onCostValueChanged);

  boost::function<void(float, float, float, float)> fun = boost::bind(&QPlanningParamWidget::showMousePosition, m_pWdgParam,
                       _1, _2, _3, _4);
  for (int i = 0; i < 2; ++i) {
    m_pWdgShow[i] = new QPlanningShowWidget(this);
    m_pWdgShow[i]->setFunPosition(fun);
//    connect(m_pWdgShow[i], &QBaseShowWidget::saveDataToFile,
//        this, &QPlanningWidget::onSaveDataToFile);
  }
  m_pWdgShow[1]->setCostType(QPlanningShowWidget::NEW_COST);
  m_nShowType = LivePlay;
  m_nShowView = LocalView;

  m_pWdgFullView = new QFullViewWidget(this);
  m_pWdgFullView->setFunPosition(fun);
  m_pWdgFullView->hide();

  namespace fs = boost::filesystem;
  m_fsPath = getenv("HOME");
  m_fsPath /= "NewPlanningData";

  char time_str[64] = {0};
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);

  sprintf(time_str, "%04d%02d%02d_%02d%02d%02d",
          utcTime->tm_year + 1900,
          utcTime->tm_mon + 1,
          utcTime->tm_mday,
          utcTime->tm_hour,
          utcTime->tm_min,
          utcTime->tm_sec
          );

  m_fsPath /= time_str;
  if (!fs::exists(m_fsPath)) {
    fs::create_directories(m_fsPath);
  }

//  connect(QReadDataManagerRos::instance(), SIGNAL(planningData(const debug_tool::ads_PlanningData4Debug &)),
//          this, SLOT(onParsePlanningData(const debug_tool::ads_PlanningData4Debug &)));

  QReadDataManagerRos::instance()->start_subscribe();

  m_nReplaySpeedIndex = 1;
//  memset(m_dCostValue, 0, sizeof(double) * QPlanningCostWidget::Count);
}
