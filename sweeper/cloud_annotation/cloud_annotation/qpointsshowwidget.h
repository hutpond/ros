#ifndef QPOINTSSHOWWIDGET_H
#define QPOINTSSHOWWIDGET_H

#include <QOpenGLWidget>
#include <QVector3D>

class QCloudPoints;

class QPointsShowWidget : public QOpenGLWidget
{
public:
  explicit QPointsShowWidget(QCloudPoints &, QWidget *);

protected:
  virtual void initializeGL() override;
  virtual void resizeGL(int w, int h) override;
  virtual void paintGL() override;
  virtual void mousePressEvent(QMouseEvent *) override;
  virtual void mouseMoveEvent(QMouseEvent *) override;
  virtual void mouseReleaseEvent(QMouseEvent *) override;
  virtual void wheelEvent(QWheelEvent *) override;

private:
  QCloudPoints &m_rObjCloudPoints;

  QPointF m_ptfPress;
  QPointF m_ptfMove;

  QVector3D m_translate;
  QVector3D m_rotation;
  QVector3D m_scale;
};

#endif // QPOINTSSHOWWIDGET_H
