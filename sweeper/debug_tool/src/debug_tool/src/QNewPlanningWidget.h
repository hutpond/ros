#ifndef QNEWPLANNINGWIDGET_H
#define QNEWPLANNINGWIDGET_H

#include "QBaseWidget.h"
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"

class QNewPlanningWidget : public QBaseWidget
{
public:
  explicit QNewPlanningWidget(QWidget *parent = Q_NULLPTR);

protected:
  void timerEvent(QTimerEvent *);

protected slots:
  void onParsePlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &);

protected:
  void setPlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &, const QString &);
  void saveDataToJsonFile(const std::string &, const debug_ads_msgs::ads_msgs_planning_debug_frame &);
  bool readFromJsonFile(const std::string &, debug_ads_msgs::ads_msgs_planning_debug_frame &);
  void parseDataFromJson(const Json::Value &, debug_ads_msgs::ads_msgs_planning_debug_frame &);
};

#endif // QNEWPLANNINGWIDGET_H
