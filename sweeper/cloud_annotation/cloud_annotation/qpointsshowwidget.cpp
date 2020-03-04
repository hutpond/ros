#include "qpointsshowwidget.h"

#include <cfloat>
#include <GL/glu.h>
#include <GL/glut.h>
#include <QOpenGLFunctions_2_1>
#include <QMouseEvent>
#include <QDebug>
#include "qcloudpoints.h"
#include "globalfunction.h"
#include "gl2ps.h"

QPointsShowWidget::QPointsShowWidget(QCloudPoints &points, QWidget *parent)
  : m_rObjCloudPoints(points)
  , QOpenGLWidget(parent)
{
  bgcolor_ = QColor(0xA0, 0xA0, 0xA0, 0x0);

  ortho_ = true;
  reset();

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

void QPointsShowWidget::reset()
{
  xRot_ = yRot_ = zRot_ = 0.0;
  xShift_ = yShift_ = zShift_ = xVPShift_ = yVPShift_ = 0.0;
  xScale_ = yScale_ = zScale_ = 1.0;
  zoom_ = 1;

  this->update();
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
  glViewport(0, 0, w, h);
}

void QPointsShowWidget::paintGL()
{
  glClearColor(bgcolor_.red() / float(0xFF),
               bgcolor_.green() / float(0xFF),
               bgcolor_.blue() / float(0xFF),
               bgcolor_.alpha() / float(0xFF)
               );
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  applyLights();

//  glRotatef( -90, 1.0, 0.0, 0.0 );
//  glRotatef( 0.0, 0.0, 1.0, 0.0 );
//  glRotatef( 0.0, 0.0, 0.0, 1.0 );


  this->draw(GL_RENDER);

  /// axis
  this->drawCoordinates();

  if (lighting_enabled_)
    glDisable(GL_NORMALIZE);

//  glMatrixMode( GL_MODELVIEW );
//  glPopMatrix();

  glFlush();
}

void QPointsShowWidget::draw(GLenum mode)
{
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

  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();

  glRotatef( xRot_-90, 1.0, 0.0, 0.0 );
  glRotatef( yRot_, 0.0, 1.0, 0.0 );
  glRotatef( zRot_, 0.0, 0.0, 1.0 );

  glScalef( zoom_ * xScale_, zoom_ * yScale_, zoom_ * zScale_ );

  glTranslatef(xShift_ - center.x, yShift_ - center.y, zShift_ - center.z);
  glTranslatef(xShift_, yShift_, zShift_);

  // matrix model
  model_matrix_.setToIdentity();
  model_matrix_.rotate(xRot_-90, 1.0, 0.0, 0.0);
  model_matrix_.rotate(yRot_, 0.0, 1.0, 0.0);
  model_matrix_.rotate(zRot_, 0.0, 0.0, 1.0);
  model_matrix_.scale(zoom_ * xScale_, zoom_ * yScale_, zoom_ * zScale_);
  model_matrix_.translate(xShift_ - center.x, yShift_ - center.y, zShift_ - center.z);
  model_matrix_.translate(xShift_, yShift_, zShift_);

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  projection_matrix_.setToIdentity();

  if (radius > 0.1) {
    if (ortho_) {
      glOrtho(-radius, +radius, -radius, +radius, 0, 20 * radius);
      projection_matrix_.ortho(-radius, +radius, -radius, +radius, 0, 20 * radius);
    }
    else {
      glFrustum( -radius, +radius, -radius, +radius, 5 * radius, 20 * radius );
      projection_matrix_.frustum( -radius, +radius, -radius, +radius, 5 * radius, 20 * radius );
    }
  }
  else {
    if (ortho_) {
      glOrtho( -1.0, 1.0, -1.0, 1.0, 10.0, 100.0 );
      projection_matrix_.ortho( -1.0, 1.0, -1.0, 1.0, 10.0, 100.0 );
    }
    else {
      glFrustum( -1.0, 1.0, -1.0, 1.0, 10.0, 100.0 );
      projection_matrix_.frustum( -1.0, 1.0, -1.0, 1.0, 10.0, 100.0 );
    }
  }
  glTranslatef( xVPShift_ * 2 * radius , yVPShift_ * 2 * radius , -7 * radius );
  projection_matrix_.translate( xVPShift_ * 2 * radius , yVPShift_ * 2 * radius , -7 * radius );

  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);

  if (lighting_enabled_)
    glEnable(GL_NORMALIZE);

  glPointSize(1.0f);

  /// cloud points
  auto &flags = m_rObjCloudPoints.selectFlag();
  auto points = m_rObjCloudPoints.points();
  const size_t sizePoints = points->size();
  glBegin(GL_POINTS);
  for (size_t i = 0; i < sizePoints; ++i) {
    if (mode == GL_SELECT) {
      glPushName(i+1);
    }
    auto &point = points->at(i);
    float factor = std::min(1.0f, (point.z - beg.z) / (end.z - beg.z) * 2.0f);
    glColor3f(factor,
              factor,
              1.0f - factor);

    QVector3D pt3d = QVector3D(point.x, point.y, point.z);
    if (ray_direction_.length() > 0) {
      double dis = pt3d.distanceToLine(ray_start_, ray_direction_);
      if (dis < 1) {
        glColor3f(1.0, 0, 0);
      }
    }
    if (*flags[i] == 1) {
      glColor3f(1.0, 0, 0);
    }

    glVertex3f(point.x, point.y, point.z);

    if (mode == GL_SELECT) {
      glPopName();
    }
  }
  glEnd();
}

void QPointsShowWidget::drawCoordinates()
{
  pcl::PointXYZ beg = m_rObjCloudPoints.begin_point();
  pcl::PointXYZ end = m_rObjCloudPoints.end_point();

  glLineWidth(1.0f);

  // x axis
  glColor3f(1.0f, 0.0f, 0.0f); //画红色的x轴
  glBegin(GL_LINES);
  glVertex3f(beg.x, beg.y, beg.z);
  glVertex3f(end.x, beg.y, beg.z);
  glEnd();

  // x name
  GLubyte xName[32] = {0};
  this->getAlphbetDotMatrix('E', xName);
  glBegin(GL_BITMAP);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glRasterPos3d(end.x, beg.y, beg.z);
  glBitmap(16, 16, 0, 0, 0, 0, xName);
  glEnd();

  // y axis
  glColor3f(0.0, 1.0, 0.0); //画绿色的y轴
  glBegin(GL_LINES);
  glVertex3f(beg.x, beg.y, beg.z);
  glVertex3f(beg.x, end.y, beg.z);
  glEnd();

  // y name
  GLubyte yName[36] = {0};
  this->getAlphbetDotMatrix('N', yName);
  glBegin(GL_BITMAP);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glRasterPos3d(beg.x, end.y, beg.z);
  glBitmap(16, 16, 0, 0, 0, 0, yName);
  glEnd();

  // z axis
  glColor3f(0.0, 0.0, 1.0); //画蓝色的z轴
  glBegin(GL_LINES);
  glVertex3f(beg.x, beg.y, beg.z);
  glVertex3f(beg.x, beg.y, end.z);
  glEnd();

  // z name
  GLubyte zName[36] = {0};
  this->getAlphbetDotMatrix('U', zName);
  glBegin(GL_BITMAP);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glRasterPos3d(beg.x, beg.y, end.z);
  glBitmap(16, 16, 0, 0, 0, 0, zName);
  glEnd();
}

void QPointsShowWidget::drawText(const QString &text)
{
  pcl::PointXYZ beg = m_rObjCloudPoints.begin_point();
  pcl::PointXYZ end = m_rObjCloudPoints.end_point();

  GLboolean b;
  GLint func;
  GLdouble v;
  glGetBooleanv(GL_ALPHA_TEST, &b);
  glGetIntegerv(GL_ALPHA_TEST_FUNC, &func);
  glGetDoublev(GL_ALPHA_TEST_REF, &v);

  glEnable (GL_ALPHA_TEST);
  glAlphaFunc (GL_NOTEQUAL, 0.0);

  //convert2screen();
  glRasterPos3d(beg.x, beg.y, beg.z);

  QVector3D pos(beg.x, beg.y, beg.z);
  drawDeviceText("East", "Courier", 20, pos, QColor(255, 0, 0), BottomLeft, 0);

  glAlphaFunc(func,v);
  glEnable(GL_ALPHA_TEST);
}

void QPointsShowWidget::selectObject(GLint x, GLint y)
{
  GLuint selectBuff[512] = {0};//创建一个保存选择结果的数组
  GLint hits, viewport[4];

  glGetIntegerv(GL_VIEWPORT, viewport); //获得viewport
  glSelectBuffer(512, selectBuff); //告诉OpenGL初始化  selectbuffer

  //进入选择模式
  glRenderMode(GL_SELECT);
  glInitNames();  //初始化名字栈
  glPushName(0);  //在名字栈中放入一个初始化名字，这里为‘0’

  glMatrixMode(GL_PROJECTION);    //进入投影阶段准备拾取
  glPushMatrix();     //保存以前的投影矩阵
  glLoadIdentity();   //载入单位矩阵

  gluPickMatrix(
        x,           // 设定我们选择框的大小，建立拾取矩阵，就是上面的公式
        viewport[3]-y,    // viewport[3]保存的是窗口的高度，窗口坐标转换为OpenGL坐标（OPengl窗口坐标系）
        4,4,              // 选择框的大小为2，2
        viewport          // 视口信息，包括视口的起始位置和大小
      );

  //投影处理，并归一化处理
  glOrtho(-2, 2, -2, 2, -2, 2);     //拾取矩阵乘以投影矩阵，这样就可以让选择框放大为和视体一样大

  draw(GL_SELECT);    // 该函数中渲染物体，并且给物体设定名字
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();  // 返回正常的投影变换

  hits = glRenderMode(GL_RENDER); // 从选择模式返回正常模式,该函数返回选择到对象的个数
  if(hits > 0) {
    QString text = QString::number(hits);
    int modelselect = 0; //离眼睛最近的物件的名字（ID）
    int n=0; double minz=selectBuff[1];
    for(int i=1;i<hits;i++)
    {
      if (selectBuff[1+i*4]<minz) {n=i;minz=selectBuff[1+i*4];}
    }
    modelselect = selectBuff[3+n*4];
    text += " " + QString("%1 ").arg(minz, 0, 'f', 2) + QString::number(modelselect);
    emit message(text);
  }
}

void QPointsShowWidget::selectRay(int x, int y, QVector3D &start, QVector3D &dir)
{
  // matrix
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  // ray
  GLdouble winX = x;
  GLdouble winY = (viewport[3] - y);
  GLdouble posX, posY, posZ;

  //获取像素对应的前裁剪面的点坐标
  float winZ = 0;
  bool bResult = gluUnProject(winX, winY, winZ, modelview, projection, viewport,
                              &posX, &posY, &posZ);
  start = QVector3D(posX, posY, posZ);

  glReadPixels( winX, viewport[3] - winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
  bResult = gluUnProject(winX, winY, winZ, modelview, projection, viewport,
                         &posX, &posY, &posZ);

  //获取像素对应的后裁剪面的点坐标
  winZ = 1.0;
  bResult = gluUnProject(winX, winY, winZ, modelview, projection, viewport,
                         &posX, &posY, &posZ);
  dir = QVector3D(posX, posY, posZ) - start;
}

void QPointsShowWidget::selectPoints(int x, int y)
{
  QRect rect(0, 0, 5, 5);
  rect.moveCenter(QPoint(x, y));

  GLdouble modelMatrix[16];
  GLdouble projMatrix[16];
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  float *data = model_matrix_.data();
  float *data2 = projection_matrix_.data();
  for (int i = 0; i < 16; ++i) {
    modelMatrix[i] = data[i];
    projMatrix[i] = data2[i];
  }

  GLdouble winx, winy, winz;
  auto &flags = m_rObjCloudPoints.selectFlag();
  auto points = m_rObjCloudPoints.points();
  const size_t sizePoints = points->size();
  for (size_t i = 0; i < sizePoints; ++i) {
    auto &point = points->at(i);
    int res = gluProject(point.x, point.x, point.z, modelMatrix, projMatrix, viewport,
                         &winx, &winy, &winz);
    if (res == GL_FALSE) {
      continue;
    }

    *flags[i] = (rect.contains(winx, winy)) ? 1 : 0;
  }
}

void QPointsShowWidget::mousePressEvent(QMouseEvent *e)
{
  lastMouseMovePosition_ = e->pos();

//  this->selectRay(lastMouseMovePosition_.x(), lastMouseMovePosition_.y(),
//                  ray_start_, ray_direction_);
//  this->update();
//  selectObject(lastMouseMovePosition_.x(), lastMouseMovePosition_.y());
  selectPoints(lastMouseMovePosition_.x(), lastMouseMovePosition_.y());
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

  setRotationMouse(bstate, 1.5, diff);
  setScaleMouse(bstate, 5, diff);
  setShiftMouse(bstate, 2, diff);
  this->update();

  lastMouseMovePosition_ = e->pos();
}

void QPointsShowWidget::wheelEvent(QWheelEvent *e)
{
  if (!mouseEnabled())
    return;

  double accel = 0.15;

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
    new_xrot = std::fmod(round(xRotation() + relyz), 360);
  if ( bstate == yrot_mstate_ )
    new_yrot = std::fmod(round(yRotation() + relx), 360);
  if ( bstate == zrot_mstate_ )
    new_zrot = std::fmod(round(zRotation() + relx), 360);

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

bool QPointsShowWidget::worldToView(const QVector3D &world, QVector3D &view)
{
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  double x, y, z;
  int res = gluProject(
        world.x(), world.y(), world.z(),
        modelview, projection, viewport,
        &x, &y, &z);
  view = QVector3D(x, y, z);

  return (res == GL_FALSE) ? false : true;
}

bool QPointsShowWidget::viewToWorld(const QVector3D &view, QVector3D &world)
{
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  double x, y, z;
  int res = gluUnProject(
        view.x(), view.y(), view.z(),
        modelview, projection, viewport,
        &x, &y, &z);
  world = QVector3D(x, y, z);

  return (res == GL_FALSE) ? false : true;
}

void QPointsShowWidget::get3Dpos(int x, int y, QVector3D *pp) {
    GLint viewport[4];
    GLfloat winX, winY, winZ;
    GLdouble object_x, object_y, object_z;
    int mouse_x = x;
    int mouse_y = y;
    glGetIntegerv(GL_VIEWPORT, viewport);

    winX = (float)mouse_x;
    winY = (float)viewport[3] - (float)mouse_y - 1.0f;
    glReadBuffer(GL_BACK);

    glReadPixels(mouse_x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
    gluUnProject((GLdouble)winX, (GLdouble)winY, (GLdouble)winZ, modelview, projection, viewport, &object_x, &object_y, &object_z);
    pp->setX(object_x);
    pp->setY(object_y);
    pp->setZ(object_z);
}

void QPointsShowWidget::getAlphbetDotMatrix(char ch, GLubyte dot[]) {
  GLubyte letters[36][36]={
    {0x0f,0x78,0x0f,0x78,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x03,0xe0,0x03,0xe0,0x03,0x60,0x03,0x60,0x03,0x60,0x03,0x60,0x03,0x60,0x03,0x40,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80},//A
    {0x0f,0xc0,0x07,0xe0,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x07,0xe0,0x07,0xe0,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x07,0xe0,0x0f,0xc0},//B
    {0x00,0xe0,0x03,0xf0,0x06,0x10,0x04,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x04,0x00,0x06,0x30,0x03,0xf0,0x00,0xe0},//C
    {0x0f,0x80,0x0f,0xc0,0x06,0x20,0x06,0x20,0x06,0x20,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x30,0x06,0x20,0x06,0x20,0x06,0x20,0x0f,0xc0,0x0f,0x80},//D
    {0x0f,0xf0,0x0f,0xf0,0x06,0x30,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x20,0x07,0xe0,0x07,0xe0,0x06,0x20,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x30,0x0f,0xf0,0x0f,0xf0},//E
    {0x0f,0x00,0x0f,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x20,0x07,0xe0,0x07,0xe0,0x06,0x20,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x30,0x0f,0xf0,0x0f,0xf0},//F
    {0x00,0xc0,0x03,0xc0,0x06,0x20,0x04,0x20,0x0c,0x70,0x0c,0x70,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x04,0x60,0x06,0x60,0x03,0xe0,0x00,0xc0},//G
    {0x1e,0x78,0x1e,0x78,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0f,0xf0,0x0f,0xf0,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x1e,0x78,0x1e,0x78},//H
    {0x03,0xc0,0x03,0xc0,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x03,0xc0,0x03,0xc0},//I
    {0x03,0x00,0x07,0x80,0x0c,0xc0,0x18,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x00,0xc0,0x01,0xe0,0x01,0xe0},//J
    {0x0f,0x30,0x0f,0x30,0x06,0x20,0x06,0x20,0x06,0x60,0x06,0x40,0x06,0x40,0x06,0xc0,0x06,0x80,0x07,0x80,0x07,0x80,0x06,0x80,0x06,0x40,0x06,0x40,0x06,0x20,0x06,0x10,0x0f,0x30,0x0f,0x30},//K
    {0x0f,0xf0,0x0f,0xf0,0x06,0x10,0x06,0x10,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x0f,0x00,0x0f,0x00},//L
    {0x0d,0xb0,0x05,0xa0,0x05,0xa0,0x05,0xa0,0x05,0xa0,0x05,0xa0,0x05,0xa0,0x05,0xa0,0x05,0xa0,0x05,0xa0,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x0e,0x70,0x0e,0x70},//M
    {0x0c,0x20,0x04,0x60,0x04,0x60,0x04,0xa0,0x04,0xa0,0x04,0xa0,0x04,0xa0,0x05,0x20,0x05,0x20,0x05,0x20,0x05,0x20,0x05,0x20,0x06,0x20,0x06,0x20,0x06,0x20,0x06,0x20,0x0e,0x30,0x0c,0x30},//N
    {0x01,0x80,0x02,0x40,0x02,0x40,0x04,0x20,0x04,0x20,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x04,0x20,0x04,0x20,0x02,0x40,0x02,0x40,0x01,0x80},//O
    {0x0f,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x06,0x00,0x07,0xe0,0x07,0xe0,0x06,0x10,0x06,0x10,0x06,0x10,0x06,0x10,0x06,0x10,0x06,0x10,0x0f,0xe0,0x0f,0xe0},//P
    {0x01,0xd0,0x02,0x70,0x02,0x60,0x04,0x20,0x06,0x60,0x0b,0xd0,0x09,0x90,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x08,0x10,0x04,0x20,0x04,0x20,0x02,0x40,0x03,0xc0,0x03,0xc0},//Q
    {0x0e,0x30,0x04,0x20,0x04,0x20,0x04,0x40,0x04,0x40,0x04,0x80,0x04,0x80,0x04,0x80,0x07,0xe0,0x07,0xe0,0x06,0x10,0x06,0x10,0x06,0x10,0x06,0x10,0x06,0x10,0x06,0x10,0x0f,0xe0,0x0f,0xe0},//R
    {0x0f,0xe0,0x0c,0x60,0x0c,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x60,0x01,0x80,0x06,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x04,0x30,0x03,0xf0,0x00,0xf0},//S
    {0x03,0xc0,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x08,0x01,0x0f,0xf0,0x0f,0xf0},//T
    {0x03,0xc0,0x07,0xe0,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x0e,0x70,0x0e,0x70},//U
    {0x00,0x80,0x01,0x40,0x01,0x40,0x01,0x40,0x01,0x40,0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x20,0x02,0x20,0x02,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x04,0x20,0x0e,0x70,0x0e,0x70},//V
    {0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x06,0x60,0x08,0x90,0x08,0x90,0x08,0x90,0x08,0x90,0x08,0x90,0x08,0x90,0x08,0x90,0x08,0x90,0x0d,0xa0,0xd0,0xa0},//W
    {0x0c,0x30,0x0c,0x30,0x04,0x20,0x04,0x20,0x04,0x20,0x02,0x40,0x02,0x40,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x02,0x40,0x02,0x40,0x04,0x20,0x04,0x20,0x04,0x20,0x0c,0x30,0x0c,0x30},//X
    {0x03,0xc0,0x03,0xc0,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x02,0x40,0x02,0x40,0x04,0x20,0x04,0x20,0x04,0x20,0x0c,0x30,0x0c,0x30},//Y
    {0x0f,0xf0,0x0f,0xf0,0x04,0x10,0x04,0x10,0x04,0x00,0x02,0x00,0x02,0x00,0x01,0x00,0x01,0x00,0x00,0x80,0x00,0x80,0x00,0x40,0x00,0x40,0x00,0x20,0x00,0x20,0xc0,0x20,0x0f,0xf0,0x0f,0xf0},//Z
    {0x07,0xe0,0x7f,0xe0,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0d,0xb0,0x0d,0xb0,0x0d,0xb0,0x0d,0xb0,0x0d,0xb0,0x0d,0xb0,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x07,0xe0,0x07,0xe0},//0
    {0x03,0xc0,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x07,0x80,0x07,0x80,0x01,0x80,0x01,0x80},//1
    {0x0f,0xf0,0x0f,0xf0,0x04,0x00,0x02,0x00,0x02,0x00,0x01,0x00,0x01,0x00,0x00,0x80,0x00,0x80,0x00,0x40,0x00,0x40,0x00,0x10,0x0c,0x10,0x0c,0x10,0x0c,0x10,0x04,0x20,0x04,0x20,0x06,0xc0},//2
    {0x03,0xc0,0x03,0xc0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x03,0xf0,0x03,0xf0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x03,0xc0,0x03,0xc0},//3
    {0x00,0xf0,0x00,0xf0,0x00,0x60,0x00,0x60,0x0f,0xe0,0x0f,0xe0,0x08,0x60,0x08,0x60,0x08,0x60,0x04,0x60,0x04,0x60,0x02,0x60,0x02,0x60,0x01,0x60,0x01,0x60,0x00,0xe0,0x00,0xe0,0x00,0x60},//4
    {0x01,0x80,0x03,0xc0,0x06,0x20,0x06,0x20,0x00,0x10,0x00,0x10,0x00,0x10,0x00,0x10,0x08,0x10,0x0f,0xe0,0x0d,0xc0,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0c,0x00,0x0f,0xf0,0x0f,0xf0},//5
    {0x03,0xc0,0x03,0xc0,0x04,0x20,0x04,0x20,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0f,0xe0,0x0d,0xc0,0x0c,0x00,0x0c,0x00,0x06,0x00,0x06,0x00,0x03,0x10,0x03,0x10,0x00,0xf0},//6
    {0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x01,0x80,0x00,0x60,0x00,0x60,0x00,0x60,0x00,0x60,0x08,0x60,0x08,0x60,0x0f,0xf0,0x0f,0xf0},//7
    {0x07,0xe0,0x7f,0xe0,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x07,0xe0,0x07,0xe0,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x07,0xe0,0x07,0xe0},//8
    {0x07,0xe0,0x7f,0xe0,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x07,0xe0,0x07,0xe0,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x0c,0x30,0x07,0xe0,0x07,0xe0}//9
  };

  if (ch >= 'A' && ch <= 'Z') {
    int index = ch - 'A';
    memcpy(dot, letters[index], 32);
  }
  else if (ch >= '0' && ch <= '9') {
    int index = ch - '0' + 26;
    memcpy(dot, letters[index], 32);
  }
}
