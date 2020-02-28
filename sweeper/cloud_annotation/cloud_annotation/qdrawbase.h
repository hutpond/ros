#ifndef QDRAWBASE_H
#define QDRAWBASE_H

#include <list>
#include <GL/gl.h>
#include <QColor>
#include <QVector3D>

class QDrawBase
{
public:
  virtual ~QDrawBase() = 0;

  virtual void draw();
  virtual void saveGLState();
  virtual void restoreGLState();

  void attach(QDrawBase*);
  void detach(QDrawBase*);
  void detachAll();

  virtual void setColor(const QColor &);
  QVector3D relativePosition(QVector3D);

protected:
  void Enable(GLenum what, GLboolean val);
  QVector3D ViewPort2World(QVector3D win, bool* err = 0);
  QVector3D World2ViewPort(QVector3D obj, bool* err = 0);

  QColor color_;
  GLdouble modelMatrix[16];
  GLdouble projMatrix[16];
  GLint viewport[4];

private:
  GLboolean ls;
  GLboolean pols;
  GLint polmode[2];
  GLfloat lw;
  GLint blsrc, bldst;
  GLdouble col[4];
  GLint pattern, factor;
  GLboolean sallowed;
  GLboolean tex2d;
  GLint matrixmode;
  GLfloat poloffs[2];
  GLboolean poloffsfill;

  std::list<QDrawBase*> dlist;
};

#endif // QDRAWBASE_H
