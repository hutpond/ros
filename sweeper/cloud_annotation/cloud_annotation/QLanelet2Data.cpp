#include "QLanelet2Data.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "GlobalDefine.h"

QLanelet2Data & QLanelet2Data::instance()
{
  static QLanelet2Data instance_;
  return instance_;
}

QLanelet2Data::QLanelet2Data()
{

}

void QLanelet2Data::setOrigin(const Point &point)
{
  origin_point_.lon = point.x;
  origin_point_.lat = point.y;
  origin_point_.ele = point.z;
}

void QLanelet2Data::setMapData(const HdMapRaw &)
{
  lanelet::LineString3d left = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 2, 1.1},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 2, 1.1},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 2, 2.1}});
  lanelet::LineString3d right = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 0, 1.2},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 0, 1.2},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 0, 2.2}});
  lanelet::Lanelet lanelet = lanelet::Lanelet(lanelet::utils::getId(), left, right);

  lanelet::LineString3d top = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 2, 1.3},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 2, 1.3},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 2, 2.3}});
  right = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 2, 0, 1.4},
                         lanelet::Point3d{lanelet::utils::getId(), 2, 1, 1.4},
                         lanelet::Point3d{lanelet::utils::getId(), 2, 2, 2.4}});
  right = right.invert();
  lanelet::LineString3d bottom = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 0, 1.5},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 0, 1.5},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 0, 2.5}});
  bottom = bottom.invert();
  left = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 0, 1.6},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 0, 1.6},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 0, 2.6}});
  lanelet::Area area = lanelet::Area(lanelet::utils::getId(), {top});


  lanelet::LineString3d trafficLight = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 3, 0, 1.7},
                                  lanelet::Point3d{lanelet::utils::getId(), 3, 1, 1.7},
                                  lanelet::Point3d{lanelet::utils::getId(), 3, 2, 2}});
  lanelet::TrafficLight::Ptr light = lanelet::TrafficLight::make(
        lanelet::utils::getId(), {}, {trafficLight});

  lanelet.addRegulatoryElement(light);

  lanelet_map_.add(lanelet);
//  lanelet_map_.add(area);
//  lanelet_map_ = *lanelet::utils::createMap({lanelet}, {area});
}

void QLanelet2Data::saveOsmMapFile(const QString &name)
{
//  lanelet::projection::Projector projector(lanelet::Origin(origin_point_));
  std::string file_name = name.toStdString();
  lanelet::write(file_name, lanelet_map_, lanelet::Origin(origin_point_));


  lanelet::LaneletMapPtr map = lanelet::load(file_name, lanelet::Origin(origin_point_));
  int index = file_name.find_last_of('/');
  file_name = file_name.substr(0, index + 1) + "A_" + file_name.substr(index + 1);
  lanelet::write(file_name, *map, lanelet::Origin(origin_point_));
}
