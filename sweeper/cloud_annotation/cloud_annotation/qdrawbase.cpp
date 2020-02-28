#include "qdrawbase.h"

#include <GL/glu.h>

#include "globalfunction.h"

QDrawBase::~QDrawBase()
{
  detachAll();
}

void QDrawBase::saveGLState()
{
  glGetBooleanv(GL_LINE_SMOOTH, &ls);
  glGetBooleanv(GL_POLYGON_SMOOTH, &pols);
  glGetFloatv(GL_LINE_WIDTH, &lw);
  glGetIntegerv(GL_BLEND_SRC, &blsrc);
  glGetIntegerv(GL_BLEND_DST, &bldst);
  glGetDoublev(GL_CURRENT_COLOR, col);
  glGetIntegerv(GL_LINE_STIPPLE_PATTERN, &pattern);
  glGetIntegerv(GL_LINE_STIPPLE_REPEAT, &factor);
  glGetBooleanv(GL_LINE_STIPPLE, &sallowed);
  glGetBooleanv(GL_TEXTURE_2D, &tex2d);
  glGetIntegerv(GL_POLYGON_MODE, polmode);
  glGetIntegerv(GL_MATRIX_MODE, &matrixmode);
  glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &poloffs[0]);
  glGetFloatv(GL_POLYGON_OFFSET_UNITS, &poloffs[1]);
  glGetBooleanv(GL_POLYGON_OFFSET_FILL, &poloffsfill);
}

void QDrawBase::restoreGLState()
{
  Enable(GL_LINE_SMOOTH, ls);
  Enable(GL_POLYGON_SMOOTH, pols);

  setDeviceLineWidth(lw);
  glBlendFunc(blsrc, bldst);
  glColor4dv(col);

  glLineStipple(factor,pattern);
  Enable(GL_LINE_STIPPLE,sallowed);
  Enable(GL_TEXTURE_2D,tex2d);
  glPolygonMode(polmode[0], polmode[1]);
  glMatrixMode(matrixmode);
  glPolygonOffset(poloffs[0], poloffs[1]);
  setDevicePolygonOffset(poloffs[0], poloffs[1]);

  Enable(GL_POLYGON_OFFSET_FILL, poloffsfill);
}

void QDrawBase::Enable(GLenum what, GLboolean val)
{
  if (val)
    glEnable(what);
  else
    glDisable(what);
}

void QDrawBase::attach(QDrawBase* dr)
{
  if ( dlist.end() == std::find( dlist.begin(), dlist.end(), dr ) )
    if (dr)
    {
      dlist.push_back(dr);
    }
}

void QDrawBase::detach(QDrawBase* dr)
{
  std::list<QDrawBase*>::iterator it = std::find(dlist.begin(), dlist.end(), dr);

  if ( it != dlist.end() )
  {
    dlist.erase(it);
  }
}
void QDrawBase::detachAll()
{
  dlist.clear();
}


//! simplified glut routine (glUnProject): windows coordinates_p --> object coordinates_p
/**
        Don't rely on (use) this in display lists !
*/
QVector3D QDrawBase::ViewPort2World(QVector3D win, bool* err)
{
  getMatrices(modelMatrix, projMatrix, viewport);

  double x, y, z;
  int res = gluUnProject(win.x(), win.y(), win.z(),
                         modelMatrix, projMatrix, viewport,
                         &x, &y, &z);
  QVector3D obj(x, y, z);

  if (err)
    *err = (res) ? false : true;
  return obj;
}

//! simplified glut routine (glProject): object coordinates_p --> windows coordinates_p
/**
        Don't rely on (use) this in display lists !
*/
QVector3D QDrawBase::World2ViewPort(QVector3D obj, bool* err)
{
  getMatrices(modelMatrix, projMatrix, viewport);

  double x, y, z;
  int res = gluProject(obj.x(), obj.y(), obj.z(),
                       modelMatrix, projMatrix, viewport,
                       &x, &y, &z);
  QVector3D win(x, y, z);

  if (err)
    *err = (res) ? false : true;
  return win;
}

/**
        Don't rely on (use) this in display lists !
*/
QVector3D QDrawBase::relativePosition(QVector3D rel)
{
  return ViewPort2World(QVector3D((rel.x() - viewport[0]) * viewport[2],
      (rel.y() - viewport[1]) * viewport[3], rel.z()));
}

void QDrawBase::draw()
{
  saveGLState();

  for (std::list<QDrawBase*>::iterator it = dlist.begin(); it!=dlist.end(); ++it)
  {
    (*it)->draw();
  }
  restoreGLState();
}

void QDrawBase::setColor(const QColor &color)
{
  color_ = color;
}
