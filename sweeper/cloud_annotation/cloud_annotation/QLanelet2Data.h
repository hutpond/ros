#ifndef QLANELET2DATA_H
#define QLANELET2DATA_H

#include <QString>
#include <lanelet2_core/LaneletMap.h>

struct HdMapRaw;

class QLanelet2Data
{
public:
  static QLanelet2Data & instance();

  void setMapData(const HdMapRaw &);
  void saveOsmMapFile(const QString &);

private:
  QLanelet2Data();

  lanelet::LaneletMap lanelet_map_;
};

#endif // QLANELET2DATA_H
