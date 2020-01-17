#include "QFrameTimeWidget.h"

#include "qwt_plot.h"
#include "qwt_plot_curve.h"
#include "qwt_plot_grid.h"

QFrameTimeWidget::QFrameTimeWidget(QWidget *parent)
  : QWidget(parent)
  , m_dCurrentMS(0.0)
{
  m_pWdgPlot = new QwtPlot(QStringLiteral(""), this);
  m_pWdgPlot->setAxisTitle(QwtPlot::xBottom, QStringLiteral("index"));
  m_pWdgPlot->setAxisTitle(QwtPlot::yLeft, QStringLiteral("time(ms)"));
  QwtPlotGrid *grid = new QwtPlotGrid;
  grid->setMajorPen(QPen(Qt::gray, 0, Qt::DotLine));
  grid->attach(m_pWdgPlot);

  m_pWdgPlot->setAxisFont(QwtPlot::xBottom, QFont("Times", 10));
  m_pWdgPlot->setAxisFont(QwtPlot::yLeft, QFont("Times", 10));

  m_pPlotCurve = new QwtPlotCurve( "Curve 1" );
  m_pPlotCurve->attach(m_pWdgPlot);
}

void QFrameTimeWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const int SPACE_X = 10;
  const int SPACE_Y = 10;
  m_pWdgPlot->setGeometry(SPACE_X, SPACE_Y, WIDTH - 2.0 * SPACE_X, HEIGHT - 2.0 * SPACE_Y);
}

void QFrameTimeWidget::showEvent(QShowEvent *)
{
  m_pWdgPlot->replot();
}

void QFrameTimeWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data)
{
  QPointF ptf;
  ptf.setX(m_points.size());
  double dCurrentMS = data.header.stamp.toSec() * 1000;
  if (qAbs<double>(m_dCurrentMS) < 1.0e-6) {
    ptf.setY(0);
  }
  else {
    ptf.setY(dCurrentMS - m_dCurrentMS);
  }
  m_dCurrentMS = dCurrentMS;
  m_points << ptf;
  m_pPlotCurve->setSamples(m_points);
  if (this->isVisible()) {
    m_pWdgPlot->replot();
  }
}

void QFrameTimeWidget::clearData()
{
  m_points.clear();
  m_dCurrentMS = 0.0;
}
