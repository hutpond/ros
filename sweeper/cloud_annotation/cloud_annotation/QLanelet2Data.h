#ifndef QLANELET2DATA_H
#define QLANELET2DATA_H

#include <QString>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/GPSPoint.h>

struct Point;
struct HdMapRaw;

class QLanelet2Data
{
public:
  static QLanelet2Data & instance();

  void setOrigin(const Point &);
  void setMapData(const HdMapRaw &);
  void saveOsmMapFile(const QString &);

private:
  QLanelet2Data();

  lanelet::LaneletMap lanelet_map_;
  lanelet::GPSPoint origin_point_;
};

#endif // QLANELET2DATA_H
