#include "qpointsshowwidget.h"

#include <QOpenGLFunctions_2_1>
#include <QMouseEvent>
#include "qcloudpoints.h"

QPointsShowWidget::QPointsShowWidget(QCloudPoints &points, QWidget *parent)
  : m_rObjCloudPoints(points)
  , QOpenGLWidget(parent)
{

}

void QPointsShowWidget::initializeGL()
{
  m_translate = QVector3D(0, 0, 0);
  m_rotation = QVector3D(0, 0, 0);

  QOpenGLFunctions_2_1 *f_2_1 = QOpenGLContext::currentContext()->versionFunctions<
      QOpenGLFunctions_2_1>();

  f_2_1->initializeOpenGLFunctions();

  f_2_1->glClearColor(0, 0, 0, 1);

  f_2_1->glEnable(GL_DEPTH_TEST);
  f_2_1->glEnable(GL_CULL_FACE);
//  f_2_1->glOrtho(-100, 100, 0, 50, -100, 100);
}

void QPointsShowWidget::resizeGL(int w, int h)
{
  const double AREA_WIDTH = 200;
  m_scale = QVector3D(AREA_WIDTH, AREA_WIDTH * h / w, AREA_WIDTH);

  QOpenGLFunctions_2_1 *f_2_1 = QOpenGLContext::currentContext()->versionFunctions<
      QOpenGLFunctions_2_1>();
  f_2_1->glViewport(0, 0, w, h);
}

void QPointsShowWidget::paintGL()
{
    QOpenGLFunctions_2_1 *f_2_1 = QOpenGLContext::currentContext()->versionFunctions<
        QOpenGLFunctions_2_1>();
    f_2_1->glClear(GL_COLOR_BUFFER_BIT);
    f_2_1->glPointSize(1.0f);

    f_2_1->glLoadIdentity();

    f_2_1->glTranslatef(m_translate.x(), m_translate.y(), m_translate.z());
    f_2_1->glScalef(1.0f / m_scale.x(), 1.0f / m_scale.y(), 1.0f / m_scale.z());
    f_2_1->glRotated(m_rotation.x(), 1, 0, 0);
    f_2_1->glRotated(m_rotation.y(), 0, 1, 0);
    f_2_1->glRotated(m_rotation.z(), 0, 0, 1);

    auto points = m_rObjCloudPoints.points();
    const size_t sizePoints = points->size();
    f_2_1->glBegin(GL_POINTS);
    f_2_1->glColor3f(1.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < sizePoints; ++i) {
      auto &point = points->at(i);
      //f_2_1->glVertex3f(point.x / m_scale.x(), point.y / m_scale.y(), point.z / m_scale.z());
      f_2_1->glVertex3f(point.x, point.y, point.z);
    }
    f_2_1->glEnd();

    f_2_1->glFlush();
}

void QPointsShowWidget::mousePressEvent(QMouseEvent *e)
{
  m_ptfPress = e->localPos();
}

void QPointsShowWidget::mouseMoveEvent(QMouseEvent *e)
{
  if ((Qt::LeftButton & e->buttons()) == Qt::NoButton) {
    return;
  }

  QPointF ptfCurrent = e->localPos();
  m_ptfMove = ptfCurrent - m_ptfPress;
  m_ptfPress = ptfCurrent;

  Qt::KeyboardModifiers modifiers = e->modifiers();
  if (modifiers == Qt::NoModifier) {
    m_translate += QVector3D(m_ptfMove.x() / width(), -m_ptfMove.y() / height(), 0);
  }
  else if (modifiers == Qt::ControlModifier) {
    m_rotation += QVector3D(-m_ptfMove.y() / height() * 360, -m_ptfMove.x() / width() * 360,0);
  }
  this->update();
}

void QPointsShowWidget::mouseReleaseEvent(QMouseEvent *)
{
}

void QPointsShowWidget::wheelEvent(QWheelEvent *e)
{
  const float RATIO_COEF = 1.05f;
  int delta = e->delta() / 120;
  m_scale /= std::pow(RATIO_COEF, delta);
  this->update();
}
