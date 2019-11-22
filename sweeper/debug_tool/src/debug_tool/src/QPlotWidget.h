#ifndef QPLOTWIDGET_H
#define QPLOTWIDGET_H

#include <QWidget>

class QPlotWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QPlotWidget(QWidget *parent = nullptr);

  void clearPoints();
  void setName(const QString &, const QString &, const QString &);
  void addPoint(QSharedPointer<QPointF>);
  void plot();

protected:
  virtual void resizeEvent(QResizeEvent *) final;
  virtual void paintEvent(QPaintEvent *) final;

signals:

public slots:

protected:
  QString m_strTitle;
  QString m_strXName;
  QString m_strYName;

  QList<QSharedPointer<QPointF>> m_points;
  QImage m_image;
};

#endif // QPLOTWIDGET_H
