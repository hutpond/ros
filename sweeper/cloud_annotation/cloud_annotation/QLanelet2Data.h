#ifndef QLANELET2DATA_H
#define QLANELET2DATA_H

#include <QString>
#include <lanelet2_core/primitives/GPSPoint.h>

struct Point;
struct HdMapRaw;

class QLanelet2Data
{
public:
  static QLanelet2Data & instance();

  void setOrigin(const Point &);
  void saveOsmMapFile(const QString &, const HdMapRaw &);

protected:
  Point calcLlaFromEnu(const Point &);

private:
  QLanelet2Data();

  lanelet::GPSPoint origin_point_;
};

#endif // QLANELET2DATA_H
