#include "QLanelet2Data.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>

QLanelet2Data & QLanelet2Data::instance()
{
  static QLanelet2Data instance_;
  return instance_;
}

QLanelet2Data::QLanelet2Data()
{

}

void QLanelet2Data::setMapData(const HdMapRaw &)
{
  lanelet::LineString3d left = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 2, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 2, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 2, 0}});
  lanelet::LineString3d right = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 0, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 0, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 0, 0}});
  lanelet::Lanelet lanelet = lanelet::Lanelet(lanelet::utils::getId(), left, right);

  lanelet::LineString3d top = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 2, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 2, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 2, 0}});
  right = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 2, 0, 0},
                         lanelet::Point3d{lanelet::utils::getId(), 2, 1, 0},
                         lanelet::Point3d{lanelet::utils::getId(), 2, 2, 0}});
  right = right.invert();
  lanelet::LineString3d bottom = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 0, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 0, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 0, 0}});
  bottom = bottom.invert();
  left = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 0, 0, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 1, 0, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 2, 0, 0}});
  lanelet::Area area = lanelet::Area(lanelet::utils::getId(), {top, right, bottom, left});


  lanelet::LineString3d trafficLight = lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 3, 0, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 3, 1, 0},
                                  lanelet::Point3d{lanelet::utils::getId(), 3, 2, 0}});
  lanelet::TrafficLight::Ptr light = lanelet::TrafficLight::make(
        lanelet::utils::getId(), {}, {trafficLight});

  lanelet.addRegulatoryElement(light);

  lanelet_map_.add(lanelet);
  lanelet_map_.add(area);
//  lanelet_map_ = *lanelet::utils::createMap({lanelet}, {area});
}

void QLanelet2Data::saveOsmMapFile(const QString &name)
{
  lanelet::io_handlers::OsmWriter writer;
  lanelet::ErrorMessages errors;
  std::string file_name = name.toStdString();
  writer.write(file_name, lanelet_map_, errors);
}
