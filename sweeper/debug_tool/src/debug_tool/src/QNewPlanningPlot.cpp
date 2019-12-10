#include "QNewPlanningPlot.h"

#include <QGridLayout>
#include "qwt_plot.h"
#include "qwt_plot_curve.h"
#include "GlobalDefine.h"

QNewPlanningPlot::QNewPlanningPlot(QWidget *parent) : QWidget(parent)
{
  m_pWdgPlot[PlotVT] = new QwtPlot(QStringLiteral("velocity"), this);
  m_pWdgPlot[PlotVT]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("time /s"));
  m_pWdgPlot[PlotVT]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("velocity m/s"));

  m_pWdgPlot[PlotAT] = new QwtPlot(QStringLiteral("acceleration"), this);
  m_pWdgPlot[PlotAT]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("time /s"));
  m_pWdgPlot[PlotAT]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("acceleration m/s2"));

  m_pWdgPlot[PlotYawT] = new QwtPlot(QStringLiteral("yaw"), this);
  m_pWdgPlot[PlotYawT]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("time /s"));
  m_pWdgPlot[PlotYawT]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("yaw rad/s"));

  m_pWdgPlot[PlotKT] = new QwtPlot(QStringLiteral("kappa"), this);
  m_pWdgPlot[PlotKT]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("time /s"));
  m_pWdgPlot[PlotKT]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("kappa"));

  m_pWdgPlot[PlotST] = new QwtPlot(QStringLiteral("s"), this);
  m_pWdgPlot[PlotST]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("time /s"));
  m_pWdgPlot[PlotST]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("s"));

  m_pWdgPlot[PlotXT] = new QwtPlot(QStringLiteral("position enu x"), this);
  m_pWdgPlot[PlotXT]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("time /s"));
  m_pWdgPlot[PlotXT]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("position enu x"));

  m_pWdgPlot[PlotYT] = new QwtPlot(QStringLiteral("position enu y"), this);
  m_pWdgPlot[PlotYT]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("time /s"));
  m_pWdgPlot[PlotYT]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("position enu y"));

  m_pWdgPlot[PlotYX] = new QwtPlot(QStringLiteral("trajectory"), this);
  m_pWdgPlot[PlotYX]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("position enu x"));
  m_pWdgPlot[PlotYX]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("position enu y"));

  for (int i = 0; i < PlotCount; ++i) {
    m_pWdgPlot[i]->setAxisFont(QwtPlot::xBottom, G_TEXT_SMALL_FONT);
    m_pWdgPlot[i]->setAxisFont(QwtPlot::yLeft, G_TEXT_SMALL_FONT);

    m_pPlotCurve[i] = new QwtPlotCurve( "Curve 1" );
    m_pPlotCurve[i]->attach(m_pWdgPlot[i]);
  }

  QGridLayout *layout = new QGridLayout;
  for (int i = 0; i < PlotCount / 2; ++i) {
    layout->addWidget(m_pWdgPlot[i], 0, i);
    layout->addWidget(m_pWdgPlot[i + PlotCount / 2], 1, i);
  }
  this->setLayout(layout);
}

void QNewPlanningPlot::setPlanningData(
    const debug_ads_msgs::ads_msgs_planning_debug_frame &frame)
{
  const auto &points = frame.trajectory_current.current_traj_points_enu;
  QVector<QPointF> ptfs[PlotCount];

  const int size_points = points.size();
  for (int i = 0; i < size_points; ++i) {
    ptfs[PlotVT].push_back(QPointF(i * 0.1, points[i].v));
    ptfs[PlotAT].push_back(QPointF(i * 0.1, points[i].a));
    ptfs[PlotYawT].push_back(QPointF(i * 0.1, points[i].theta));
    ptfs[PlotKT].push_back(QPointF(i * 0.1, points[i].kappa));
    ptfs[PlotST].push_back(QPointF(i * 0.1, points[i].s));
    ptfs[PlotXT].push_back(QPointF(i * 0.1, points[i].X));
    ptfs[PlotYT].push_back(QPointF(i * 0.1, points[i].Y));
    ptfs[PlotYX].push_back(QPointF(points[i].X, points[i].Y));
  }

  for (int i = 0; i < PlotCount; ++i) {
    m_pPlotCurve[i]->setSamples(ptfs[i]);
    if (this->isVisible()) {
      m_pWdgPlot[i]->replot();
    }
  }
}
