#ifndef QPOINTSSHOWWIDGET_H
#define QPOINTSSHOWWIDGET_H

#include <QOpenGLWidget>
#include <QVector3D>

class QCloudPoints;

class MouseState
{
public:
  MouseState(Qt::MouseButtons mb = Qt::NoButton, Qt::KeyboardModifiers km = Qt::NoModifier)
    : mb_(mb), km_(km)
  {
  }

  MouseState(Qt::MouseButton mb, Qt::KeyboardModifiers km = Qt::NoModifier)
    : mb_(mb), km_(km)
  {
  }

  bool operator==(const MouseState& ms)
  {
    return mb_ == ms.mb_ && km_ == ms.km_;
  }

  bool operator!=(const MouseState& ms)
  {
    return !operator==(ms);
  }

private:
  Qt::MouseButtons mb_;
  Qt::KeyboardModifiers km_;
};


class KeyboardState
{
public:
  KeyboardState(int key = Qt::Key_unknown, Qt::KeyboardModifiers km = Qt::NoModifier)
    : key_(key), km_(km)
  {
  }

  bool operator==(const KeyboardState& ms)
  {
    return key_ == ms.key_ && km_ == ms.km_;
  }

  bool operator!=(const KeyboardState& ms)
  {
    return !operator==(ms);
  }

private:
  int key_;
  Qt::KeyboardModifiers km_;
};



class QPointsShowWidget : public QOpenGLWidget
{
  Q_OBJECT

public:
  explicit QPointsShowWidget(QCloudPoints &, QWidget *);

  void reset();

signals:
  void message(const QString &);

protected:
  virtual void initializeGL() override;
  virtual void resizeGL(int w, int h) override;
  virtual void paintGL() override;
  virtual void mousePressEvent(QMouseEvent *) override;
  virtual void mouseMoveEvent(QMouseEvent *) override;
  virtual void mouseReleaseEvent(QMouseEvent *) override;
  virtual void wheelEvent(QWheelEvent *) override;

  void setRotationMouse(MouseState bstate, double accel, QPoint diff);
  void setScaleMouse(MouseState bstate, double accel, QPoint diff);
  void setShiftMouse(MouseState bstate, double accel, QPoint diff);

  void	setRotation( double xVal, double yVal, double zVal );
  void	setShift( double xVal, double yVal, double zVal );
  void	setViewportShift( double xVal, double yVal );
  void	setScale( double xVal, double yVal, double zVal );
  void	setZoom( double );

  double xRotation() const { return xRot_;}  //!< Returns rotation around X axis [-360..360] (some angles are equivalent)
  double yRotation() const { return yRot_;}  //!< Returns rotation around Y axis [-360..360] (some angles are equivalent)
  double zRotation() const { return zRot_;}  //!< Returns rotation around Z axis [-360..360] (some angles are equivalent)

  double xShift() const { return xShift_;} //!< Returns shift along X axis (object coordinates)
  double yShift() const { return yShift_;} //!< Returns shift along Y axis (object coordinates)
  double zShift() const { return zShift_;} //!< Returns shift along Z axis (object coordinates)

  double xViewportShift() const { return xVPShift_;} //!< Returns relative shift [-1..1] along X axis (view coordinates)
  double yViewportShift() const { return yVPShift_;} //!< Returns relative shift [-1..1] along Y axis (view coordinates)

  double xScale() const { return xScale_;} //!< Returns scaling for X values [0..inf]
  double yScale() const { return yScale_;} //!< Returns scaling for Y values [0..inf]
  double zScale() const { return zScale_;} //!< Returns scaling for Z values [0..inf]

  double zoom() const { return zoom_;} //!< Returns zoom (0..inf)

  // mouse
  void assignMouse(MouseState xrot, MouseState yrot, MouseState zrot,
                   MouseState xscale, MouseState yscale, MouseState zscale,
                   MouseState zoom, MouseState xshift, MouseState yshift);
  bool mouseEnabled() const {return mouse_input_enabled_;}

  // light
  void applyLight(unsigned);
  void applyLights();
  GLenum lightEnum(unsigned);
  void setLightShift( double xVal, double yVal, double zVal, unsigned int idx = 0 );
  void enableLighting(bool val = true);
  void disableLighting(bool val = true);

  //
  void setMaterialComponent(GLenum property, double r, double g, double b, double a = 1.0);
  void setMaterialComponent(GLenum property, double intensity);
  void setShininess(double exponent);
  void setLightComponent(GLenum property, double r, double g, double b, double a = 1.0, unsigned light=0);
  void setLightComponent(GLenum property, double intensity, unsigned light=0);

  // select point
  void get3Dpos(int, int, QVector3D *);

  // axis
  void drawCoordinates();
  void getAlphbetDotMatrix(char, GLubyte []);

private:
  QCloudPoints &m_rObjCloudPoints;

  // bkg color
  QColor bgcolor_;

  // opengl state
  bool ortho_;

  GLdouble xRot_, yRot_, zRot_;
  GLdouble xShift_, yShift_, zShift_;
  GLdouble zoom_;
  GLdouble xScale_, yScale_, zScale_;
  GLdouble xVPShift_, yVPShift_;

  // mouse
  QPoint lastMouseMovePosition_;
  bool mpressed_;

  MouseState xrot_mstate_,
  yrot_mstate_,
  zrot_mstate_,
  xscale_mstate_,
  yscale_mstate_,
  zscale_mstate_,
  zoom_mstate_,
  xshift_mstate_,
  yshift_mstate_;

  bool mouse_input_enabled_;

  // lignt
  struct Light
  {
    Light() : unlit(true){}
    bool unlit;
    QVector3D rot;
    QVector3D shift;
  };
  std::vector<Light> lights_;
  bool lighting_enabled_;

  // text
  GLuint base_;
};

#endif // QPOINTSSHOWWIDGET_H
