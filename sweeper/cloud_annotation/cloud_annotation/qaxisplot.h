#ifndef QAXISPLOT_H
#define QAXISPLOT_H

#include "qdrawbase.h"
#include "globalfunction.h"

#include <vector>
#include <QFont>

class QAxisPlot : public QDrawBase
{
public:
  explicit QAxisPlot(QVector3D, QVector3D, COORDSTYLE = BOX);
  ~QAxisPlot();

  void init(QVector3D beg = QVector3D(0,0,0), QVector3D end = QVector3D(0,0,0));
  //! Set style for the coordinate system (NOCOORD, FRAME or BOX)
  void setStyle(COORDSTYLE s,	AXIS frame_1 = X1,
                AXIS frame_2 = Y1,
                AXIS frame_3 = Z1);
  COORDSTYLE style() const { return style_;} 	//!< Return style oft the coordinate system
  void setPosition(QVector3D first, QVector3D second); //!< first == front_left_bottom, second == back_right_top

  void setAxesColor(QColor val); //!< Set common color for all axes
  //! Set common font for all axis numberings
  void setNumberFont(QString const& family, int pointSize, int weight = QFont::Normal, bool italic = false);
  //! Set common font for all axis numberings
  void setNumberFont(QFont const& font);
  //! Set common color for all axis numberings
  void setNumberColor(QColor val);
  void setStandardScale(); //!< Sets an linear axis with real number items

  void adjustNumbers(int val); //!< Fine tunes distance between axis numbering and axis body
  void adjustLabels(int val); //!< Fine tunes distance between axis label and axis body

  //! Sets color for the grid lines
  void setGridLinesColor(QColor val) {gridlinecolor_ = val;}

  //! Set common font for all axis labels
  void setLabelFont(QString const& family, int pointSize, int weight = QFont::Normal, bool italic = false);
  //! Set common font for all axis labels
  void setLabelFont(QFont const& font);
  //! Set common color for all axis labels
  void setLabelColor(QColor val);

  //! Set line width for tic marks and axes
  void setLineWidth(double val, double majfac = 0.9, double minfac = 0.5);
  //! Set length for tic marks
  void setTicLength(double major, double minor);

  //! Switch autoscaling of axes
  void setAutoScale(bool val = true);

  QVector3D first() const { return first_;}
  QVector3D second() const { return second_;}

  void setAutoDecoration(bool val = true) {autodecoration_ = val;}
  bool autoDecoration() const { return autodecoration_;}

  void setLineSmooth(bool val = true) {smooth_ = val;} //!< draw smooth axes
  bool lineSmooth() const {return smooth_;}            //!< smooth axes ?

  void draw();

  //! Defines whether a grid between the major and/or minor tics should be drawn
  void setGridLines(bool majors, bool minors, int sides = Qwt3D::NOSIDEGRID);
  int grids() const {return sides_;} //!< Returns grids switched on

  //! The vector of all12 axes - use them to set axis properties individually.
  std::vector<Axis> axes;


private:
  void destroy();

  QVector3D first_, second_;
  COORDSTYLE style_;

  QColor gridlinecolor_;

  bool smooth_;

  void chooseAxes();
  void autoDecorateExposedAxis(Axis& ax, bool left);
  void drawMajorGridLines(); //!< Draws a grid between the major tics on the site
  void drawMinorGridLines(); //!< Draws a grid between the minor tics on the site
  void drawMajorGridLines(Qwt3D::Axis&, Qwt3D::Axis&); //! Helper
  void drawMinorGridLines(Qwt3D::Axis&, Qwt3D::Axis&); //! Helper
  void recalculateAxesTics();

  bool autodecoration_;
  bool majorgridlines_, minorgridlines_;
  int  sides_;
};

#endif // QAXISPLOT_H
