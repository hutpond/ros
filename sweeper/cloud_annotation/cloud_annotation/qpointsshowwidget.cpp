#include "qpointsshowwidget.h"

#include <cfloat>
#include <QOpenGLFunctions_2_1>
#include <QMouseEvent>
#include "qcloudpoints.h"

QPointsShowWidget::QPointsShowWidget(QCloudPoints &points, QWidget *parent)
  : m_rObjCloudPoints(points)
  , QOpenGLWidget(parent)
{
  ortho_ = true;

  xRot_ = yRot_ = zRot_ = 0.0;
  xShift_ = yShift_ = zShift_ = xVPShift_ = yVPShift_ = 0.0;
  xScale_ = yScale_ = zScale_ = 1.0;
  zoom_ = 1;

  setFocusPolicy(Qt::StrongFocus);
  assignMouse(
        MouseState(Qt::LeftButton, Qt::ControlModifier),
        MouseState(Qt::LeftButton, Qt::ShiftModifier),
        MouseState(Qt::LeftButton, Qt::ControlModifier),

        MouseState(Qt::LeftButton, Qt::AltModifier),
        MouseState(Qt::LeftButton, Qt::AltModifier),
        MouseState(Qt::LeftButton, Qt::AltModifier | Qt::ShiftModifier),

        MouseState(Qt::LeftButton, Qt::AltModifier | Qt::ControlModifier),

        Qt::LeftButton,
        Qt::LeftButton
        );
  mouse_input_enabled_ = true;

  lighting_enabled_ = false;
  disableLighting();
  lights_ = std::vector<Light>(8);
}

void QPointsShowWidget::initializeGL()
{
  QOpenGLFunctions_2_1 *f_2_1 = QOpenGLContext::currentContext()->versionFunctions<
      QOpenGLFunctions_2_1>();

  f_2_1->initializeOpenGLFunctions();

  f_2_1->glEnable(GL_BLEND);
  f_2_1->glEnable(GL_DEPTH_TEST);
  f_2_1->glShadeModel(GL_SMOOTH);

  // Set up the lights
  disableLighting();

  GLfloat whiteAmb[4] = {1.0, 1.0, 1.0, 1.0};
  setLightShift(0, 0, 3000);
  f_2_1->glEnable(GL_COLOR_MATERIAL);

  f_2_1->glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  f_2_1->glLightModelfv(GL_LIGHT_MODEL_AMBIENT, whiteAmb);

  setMaterialComponent(GL_DIFFUSE, 1.0);
  setMaterialComponent(GL_SPECULAR, 0.3);
  setMaterialComponent(GL_SHININESS, 5.0);
  setLightComponent(GL_DIFFUSE, 1.0);
  setLightComponent(GL_SPECULAR, 1.0);
}

void QPointsShowWidget::resizeGL(int w, int h)
{
  QOpenGLFunctions_2_1 *f_2_1 = QOpenGLContext::currentContext()->versionFunctions<
      QOpenGLFunctions_2_1>();
  f_2_1->glViewport(0, 0, w, h);
}

void QPointsShowWidget::paintGL()
{
  QOpenGLFunctions_2_1 *f_2_1 = QOpenGLContext::currentContext()->versionFunctions<
      QOpenGLFunctions_2_1>();

  //glClearColor(bgcolor_.r, bgcolor_.g, bgcolor_.b, bgcolor_.a);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  applyLights();

  glRotatef( -90, 1.0, 0.0, 0.0 );
  glRotatef( 0.0, 0.0, 1.0, 0.0 );
  glRotatef( 0.0, 0.0, 0.0, 1.0 );

//  if (displaylegend_)
//  {
//    legend_.draw();
//  }
//  title_.setRelPosition(titlerel_, titleanchor_);
//  title_.draw();

    pcl::PointXYZ beg = m_rObjCloudPoints.begin_point();
    pcl::PointXYZ end = m_rObjCloudPoints.end_point();

    pcl::PointXYZ center = pcl::PointXYZ(
          beg.x + (end.x - beg.x) / 2,
          beg.y + (end.y - beg.y) / 2,
          beg.z + (end.z - beg.z) / 2
          );
    double radius = std::sqrt(
          (end.x - beg.x) * (end.x - beg.x) +
          (end.y - beg.y) * (end.y - beg.y) +
          (end.z - beg.z) * (end.z - beg.z)
          );

  glLoadIdentity();

  glRotatef( xRot_-90, 1.0, 0.0, 0.0 );
  glRotatef( yRot_, 0.0, 1.0, 0.0 );
  glRotatef( zRot_, 0.0, 0.0, 1.0 );

  glScalef( zoom_ * xScale_, zoom_ * yScale_, zoom_ * zScale_ );

  glTranslatef(xShift_ - center.x, yShift_ - center.y, zShift_ - center.z);
  f_2_1->glTranslatef(xShift_, yShift_, zShift_);

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();

  if (radius > 0.1)
  {
    if (ortho_)
    {
      glOrtho( -radius, +radius, -radius, +radius, 0, 40 * radius);
    }
    else
    {
      glFrustum( -radius, +radius, -radius, +radius, 5 * radius, 400 * radius );
    }
  }
  else
  {
    if (ortho_)
      glOrtho( -1.0, 1.0, -1.0, 1.0, 10.0, 100.0 );
    else
      glFrustum( -1.0, 1.0, -1.0, 1.0, 10.0, 100.0 );
  }
  glTranslatef( xVPShift_ * 2 * radius , yVPShift_ * 2 * radius , -7 * radius );

  if (lighting_enabled_)
    glEnable(GL_NORMALIZE);

  f_2_1->glPointSize(1.0f);

  auto points = m_rObjCloudPoints.points();
  const size_t sizePoints = points->size();
  f_2_1->glBegin(GL_POINTS);
  for (size_t i = 0; i < sizePoints; ++i) {
    auto &point = points->at(i);
    float factor = std::min(1.0f, (point.z - beg.z) / (end.z - beg.z) * 2.0f);
    f_2_1->glColor3f(factor,
                     factor,
                     1.0f - factor);
    f_2_1->glVertex3f(point.x, point.y, point.z);
  }
  f_2_1->glEnd();

  if (lighting_enabled_)
    glDisable(GL_NORMALIZE);

  glMatrixMode( GL_MODELVIEW );
  glPopMatrix();

}

void QPointsShowWidget::mousePressEvent(QMouseEvent *e)
{
  lastMouseMovePosition_ = e->pos();
  mpressed_ = true;
}

void QPointsShowWidget::mouseReleaseEvent(QMouseEvent *)
{
  mpressed_ = false;
}

void QPointsShowWidget::mouseMoveEvent(QMouseEvent *e)
{
  if (!mpressed_ || !mouseEnabled())
  {
    e->ignore();
    return;
  }

  MouseState bstate(e->buttons(), e->modifiers());

  QPoint diff = e->pos() - lastMouseMovePosition_;

  setRotationMouse(bstate, 3, diff);
  setScaleMouse(bstate, 5, diff);
  setShiftMouse(bstate, 2, diff);
  this->update();

  lastMouseMovePosition_ = e->pos();
}

void QPointsShowWidget::wheelEvent(QWheelEvent *e)
{
  if (!mouseEnabled())
    return;

  double accel = 0.1;

  constexpr double WHEEL_DELTA = 120;
  double step =  accel * e->delta() / WHEEL_DELTA ;
  step = exp(step)-1;

  if ( e->modifiers() & Qt::ShiftModifier )
    setScale(xScale(), yScale(), qMax<double>(0.0,zScale() + step));
  else
    setZoom(qMax<double>(0.0, zoom() + step));
  this->update();
}

void QPointsShowWidget::assignMouse(MouseState xrot, MouseState yrot, MouseState zrot,
				    MouseState xscale, MouseState yscale, MouseState zscale,
				    MouseState zoom, MouseState xshift, MouseState yshift)
{
  xrot_mstate_   =  xrot;
  yrot_mstate_   =  yrot;
  zrot_mstate_   =  zrot;
  xscale_mstate_ =  xscale;
  yscale_mstate_ =  yscale;
  zscale_mstate_ =  zscale;
  zoom_mstate_   =  zoom;
  xshift_mstate_ =  xshift;
  yshift_mstate_ =  yshift;
}

void QPointsShowWidget::setRotationMouse(MouseState bstate, double accel, QPoint diff)
{
  // Rotation
  double w = qMax<double>(1,width());
  double h = qMax<double>(1,height());

  double relx = accel*360 * diff.x() / w;
  double relyz = accel*360 * diff.y() / h;

  double new_xrot = xRotation();
  double new_yrot = yRotation();
  double new_zrot = zRotation();

  if ( bstate == xrot_mstate_ )
    new_xrot = round(xRotation() + relyz);
  if ( bstate == yrot_mstate_ )
    new_yrot = round(yRotation() + relx);
  if ( bstate == zrot_mstate_ )
    new_zrot = round(zRotation() + relx);

  setRotation(new_xrot, new_yrot, new_zrot);
}

void QPointsShowWidget::setScaleMouse(MouseState bstate, double accel, QPoint diff)
{
  // Scale
  double w = qMax<double>(1,width());
  double h = qMax<double>(1,height());

  double relx = diff.x() * accel / w; relx = exp(relx) - 1;
  double relyz = diff.y() * accel / h; relyz = exp(relyz) - 1;

  double new_xscale = xScale();
  double new_yscale = yScale();
  double new_zscale = zScale();

  if ( bstate == xscale_mstate_)
    new_xscale = qMax<double>(0.0,xScale() + relx);
  if ( bstate == yscale_mstate_)
    new_yscale = qMax<double>(0.0,yScale() - relyz);
  if ( bstate == zscale_mstate_)
    new_zscale = qMax<double>(0.0,zScale() - relyz);

  setScale(new_xscale, new_yscale, new_zscale);

  if ( bstate == zoom_mstate_)
    setZoom(qMax<double>(0.0,zoom() - relyz));
}

void QPointsShowWidget::setShiftMouse(MouseState bstate, double accel, QPoint diff)
{
  // Shift
  double w = qMax<double>(1,width());
  double h = qMax<double>(1,height());

  double relx = diff.x() * accel / w;
  double relyz = diff.y() * accel / h;

  double new_xshift = xViewportShift();
  double new_yshift = yViewportShift();

  if ( bstate == xshift_mstate_)
    new_xshift = xViewportShift() + relx;
  if ( bstate == yshift_mstate_)
    new_yshift = yViewportShift() - relyz;

  setViewportShift(new_xshift, new_yshift);
}

void QPointsShowWidget::setRotation( double xVal, double yVal, double zVal )
{
  if (xRot_ == xVal && yRot_ == yVal && zRot_ == zVal)
    return;

  xRot_ = xVal;
  yRot_ = yVal;
  zRot_ = zVal;

  //	updateGL();
  //	emit rotationChanged(xVal, yVal, zVal);
}

/**
  Set the shift in object (world) coordinates.
        \param xVal shift along (world) X axis
        \param yVal shift along (world) Y axis
        \param zVal shift along (world) Z axis
        \see setViewportShift()
*/
void QPointsShowWidget::setShift( double xVal, double yVal, double zVal )
{
  if (xShift_ == xVal && yShift_ == yVal && zShift_ == zVal)
    return;

  xShift_ = xVal;
  yShift_ = yVal;
  zShift_ = zVal;
  //	updateGL();
  //	emit shiftChanged(xVal, yVal, zVal);
}

/**
  Performs shifting along screen axes.
  The shift moves points inside a sphere,
  which encloses the unscaled and unzoomed data
        by multiples of the spheres diameter

	\param xVal shift along (view) X axis
	\param yVal shift along (view) Y axis
	\see setShift()
*/
void QPointsShowWidget::setViewportShift( double xVal, double yVal )
{
  if (xVPShift_ == xVal && yVPShift_ == yVal)
    return;

  xVPShift_ = xVal;
  yVPShift_ = yVal;

  //	updateGL();
  //	emit vieportShiftChanged(xVPShift_, yVPShift_);
}

/**
  Set the scale in object (world) coordinates.
        \param xVal scaling for X values
        \param yVal scaling for Y values
        \param zVal scaling for Z values

        A respective value of 1 represents no scaling;
*/
void QPointsShowWidget::setScale( double xVal, double yVal, double zVal )
{
  if (xScale_ == xVal && yScale_ == yVal && zScale_ == zVal)
    return;

  xScale_ = (xVal < DBL_EPSILON ) ? DBL_EPSILON : xVal;
  yScale_ = (yVal < DBL_EPSILON ) ? DBL_EPSILON : yVal;
  zScale_ = (zVal < DBL_EPSILON ) ? DBL_EPSILON : zVal;

  //	updateGL();
  //	emit scaleChanged(xVal, yVal, zVal);
}

/**
  Set the (zoom in addition to scale).
        \param val zoom value (value == 1 indicates no zooming)
*/
void QPointsShowWidget::setZoom( double val )
{
  if (zoom_ == val)
    return;

  zoom_ = (val < DBL_EPSILON ) ? DBL_EPSILON : val;
  //updateGL();
  //        emit zoomChanged(val);
}

void QPointsShowWidget::applyLight(unsigned light)
{
  if (lights_[light].unlit)
    return;

  glEnable(lightEnum(light));
  glLoadIdentity();

  glRotatef( lights_[light].rot.x()-90, 1.0, 0.0, 0.0 );
  glRotatef( lights_[light].rot.y()   , 0.0, 1.0, 0.0 );
  glRotatef( lights_[light].rot.z()   , 0.0, 0.0, 1.0 );
  GLfloat lightPos[4] = { lights_[light].shift.x(), lights_[light].shift.y(),
                          lights_[light].shift.z(), 1.0};
  GLenum le = lightEnum(light);
  glLightfv(le, GL_POSITION, lightPos);
}

void QPointsShowWidget::applyLights()
{
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  for (unsigned i=0; i<8; ++i)
  {
    applyLight(i);
  }
  glPopMatrix();
}

void QPointsShowWidget::setLightShift( double xVal, double yVal, double zVal, unsigned light )
{
  if (light>7)
    return;
  lights_[light].shift.setX(xVal);
  lights_[light].shift.setY(yVal);
  lights_[light].shift.setZ(zVal);
}

void QPointsShowWidget::enableLighting(bool val)
{
  if (lighting_enabled_ == val)
    return;

  lighting_enabled_ = val;
  makeCurrent();
  if (val)
    glEnable(GL_LIGHTING);
  else
    glDisable(GL_LIGHTING);
}

void QPointsShowWidget::disableLighting(bool val)
{
  enableLighting(!val);
}

GLenum QPointsShowWidget::lightEnum(unsigned idx)
{
  switch(idx) {
  case 0:
        return GL_LIGHT0;
  case 1:
        return GL_LIGHT1;
  case 2:
        return GL_LIGHT2;
  case 3:
        return GL_LIGHT3;
  case 4:
        return GL_LIGHT4;
  case 5:
        return GL_LIGHT5;
  case 6:
        return GL_LIGHT6;
  case 7:
        return GL_LIGHT7;
  default:
        return GL_LIGHT0;
  }
}

/**
  Sets GL material properties
*/
void QPointsShowWidget::setMaterialComponent(GLenum property, double r, double g, double b, double a)
{
  GLfloat rgba[4] = {(GLfloat)r, (GLfloat)g, (GLfloat)b, (GLfloat)a};
  makeCurrent();
  glMaterialfv(GL_FRONT_AND_BACK, property, rgba);
}

/**
  This function is for convenience. It sets GL material properties with the equal r,g,b values
  and a blending alpha with value 1.0
*/
void QPointsShowWidget::setMaterialComponent(GLenum property, double intensity)
{
  setMaterialComponent(property,intensity,intensity,intensity,1.0);
}

/**
  Sets GL shininess
*/
void QPointsShowWidget::setShininess(double exponent)
{
  makeCurrent();
  glMaterialf(GL_FRONT, GL_SHININESS, exponent);
}

/**
  Sets GL light properties for light 'light'
*/
void QPointsShowWidget::setLightComponent(GLenum property, double r, double g, double b, double a, unsigned light)
{
  GLfloat rgba[4] = {(GLfloat)r, (GLfloat)g, (GLfloat)b, (GLfloat)a};
  makeCurrent();
  glLightfv(lightEnum(light), property, rgba);
}

/**
  This function is for convenience. It sets GL light properties with the equal r,g,b values
  and a blending alpha with value 1.0
*/
void QPointsShowWidget::setLightComponent(GLenum property, double intensity, unsigned light)
{
  setLightComponent(property,intensity,intensity,intensity,1.0, lightEnum(light));
}
