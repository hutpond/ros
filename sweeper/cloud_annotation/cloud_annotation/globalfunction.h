#ifndef GLOBALFUNCTION_H
#define GLOBALFUNCTION_H

#include <GL/gl.h>
#include <QVector3D>

enum ANCHOR
{
  BottomLeft,
  BottomRight,
  BottomCenter,
  TopLeft,
  TopRight,
  TopCenter,
  CenterLeft,
  CenterRight,
  Center
};

enum AXIS
{
  X1 = 0,   //!<  1st x-axis
  X2 = 3,   //!<  2nd x-axis
  X3 = 4,   //!<  3th x-axis
  X4 = 5,   //!<  4th x-axis
  Y1 = 1,   //!<  1st y-axis
  Y2 = 8,   //!<  2nd y-axis
  Y3 = 7,   //!<  3th y-axis
  Y4 = 6,   //!<  4th y-axis
  Z1 = 2,   //!<  1st z-axis
  Z2 = 9,   //!<  2nd z-axis
  Z3 = 11,  //!<  3th z-axis
  Z4 = 10   //!<  4th z-axis
};

enum COORDSTYLE
{
  NOCOORD, //!< Coordinate system is not visible
  BOX,     //!< Boxed
  FRAME		 //!< Frame - 3 visible axes
};

GLint setDeviceLineWidth(GLfloat val);
GLint setDevicePointSize(GLfloat val);
GLint drawDevicePixels(GLsizei width, GLsizei height, GLenum format, GLenum type, const void *pixels);
GLint drawDeviceText(const char* str, const char* fontname, int fontsize, QVector3D pos, QColor color, ANCHOR align, double gap);
void setDevicePolygonOffset(GLfloat factor, GLfloat units);

void getMatrices(GLdouble* modelMatrix, GLdouble* projMatrix, GLint* viewport);
bool ViewPort2World(double& objx, double& objy, double& objz, double winx, double winy, double winz);
bool World2ViewPort(double& winx, double& winy, double& winz, double objx, double objy, double objz);


#endif // GLOBALFUNCTION_H
