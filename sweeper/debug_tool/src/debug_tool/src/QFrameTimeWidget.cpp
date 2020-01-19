#include "QFrameTimeWidget.h"

#include "qwt_plot.h"
#include "qwt_plot_curve.h"
#include "qwt_plot_grid.h"

QFrameTimeWidget::QFrameTimeWidget(QWidget *parent)
  : QWidget(parent)
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

void QFrameTimeWidget::setPlanningData(quint64 preMillSecond, const debug_tool::ads_PlanningData4Debug &data)
{
  quint64 nCurrentMS = static_cast<quint64>(data.header.stamp.toSec() * 1000);
  QPair<quint64, quint64> pair_millsecond;
  pair_millsecond.second = nCurrentMS;

  const int size_item = m_listMillSecond.size();
  if (size_item == 0) {
    pair_millsecond.first = preMillSecond;
    m_listMillSecond.push_back(pair_millsecond);
  }
  else if (nCurrentMS > m_listMillSecond[size_item - 1].second) {
    pair_millsecond.first = m_listMillSecond[size_item - 1].second;
    m_listMillSecond.push_back(pair_millsecond);
  }
  else if (nCurrentMS < m_listMillSecond[0].second) {
    pair_millsecond.first = preMillSecond;
    m_listMillSecond.push_front(pair_millsecond);
  }
  else {
    for (int i = 1; i < size_item; ++i) {
      if (nCurrentMS == m_listMillSecond[i - 1].second || nCurrentMS == m_listMillSecond[i].second) {
        break;
      }
      else if (nCurrentMS > m_listMillSecond[i - 1].second && nCurrentMS < m_listMillSecond[i].second) {
        if (preMillSecond != 0) {
          pair_millsecond.first = preMillSecond;
        }
        else {
          pair_millsecond.first = m_listMillSecond[i - 1].second;
        }
        m_listMillSecond[i].first = pair_millsecond.second;
        m_listMillSecond.insert(i, pair_millsecond);
      }
    }
  }

  QVector<QPointF> points;
  for (int i = 1; i < m_listMillSecond.size(); ++i) {
    points << QPointF(i, m_listMillSecond[i].second - m_listMillSecond[i].first);
  }
  m_pPlotCurve->setSamples(points);
  if (this->isVisible()) {
    m_pWdgPlot->replot();
  }
}

void QFrameTimeWidget::clearData()
{
  m_listMillSecond.clear();
}
