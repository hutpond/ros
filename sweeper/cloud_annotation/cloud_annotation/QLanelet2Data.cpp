#include "QLanelet2Data.h"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "GlobalDefine.h"
#include "gps.h"
#include "ExampleHelpers.h"

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

void QLanelet2Data::saveOsmMapFile(const QString &name, const HdMapRaw &hdmap)
{
  // road & area
  lanelet::Lanelets lanelets;
  lanelet::Areas areas;
  for (const auto &segment : hdmap.road_segments) {
    if (segment.type == RoadSegment::ROAD) {
      for (const auto &road_item : segment.roads) {
        const auto &left_side = road_item.second.left_side;
        lanelet::LineString3d left = lanelet::LineString3d(lanelet::utils::getId());
        for (const auto &point : left_side) {
          Point lla = this->calcLlaFromEnu(point);
          left.push_back(
                lanelet::Point3d{lanelet::utils::getId(), lla.x, lla.y, lla.z});
        }

        const auto &right_side = road_item.second.right_side;
        lanelet::LineString3d right = lanelet::LineString3d(lanelet::utils::getId());
        for (const auto &point : right_side) {
          Point lla = this->calcLlaFromEnu(point);
          right.push_back(
                lanelet::Point3d{lanelet::utils::getId(), lla.x, lla.y, lla.z});
        }

        lanelet::Lanelet lanelet = lanelet::Lanelet(lanelet::utils::getId(), left, right);
        lanelets.push_back(lanelet);
      }
    }
    else {
      lanelet::Area area = lanelet::Area(lanelet::utils::getId());
      lanelet::LineStrings3d bound = lanelet::LineStrings3d(lanelet::utils::getId());
      for (const auto &road_item : segment.roads) {
        const auto &left_side = road_item.second.left_side;
        lanelet::LineString3d line = lanelet::LineString3d(lanelet::utils::getId());
        for (const auto &point : left_side) {
          Point lla = this->calcLlaFromEnu(point);
          line.push_back(
                lanelet::Point3d{lanelet::utils::getId(), lla.x, lla.y, lla.z});
        }
        bound.push_back(line);
      }
      area.setOuterBound(bound);
      std::vector<lanelet::LineStrings3d> bounds;
      area.setInnerBounds(bounds);
      areas.push_back(area);
    }
  }

  // traffic light
  for (const auto &light : hdmap.traffic_lights) {

  }

  auto map = lanelet::utils::createMap(lanelets, areas);

  //lanelet::projection::Projector projector(lanelet::Origin(origin_point_));
  std::string file_name = name.toStdString() + ".osm";
  lanelet::write(file_name, *map, lanelet::Origin(origin_point_));

  file_name = name.toStdString() + ".bin";
  lanelet::write(file_name, *map, lanelet::Origin(origin_point_));
  lanelet::LaneletMapPtr map_load = lanelet::load(file_name, lanelet::Origin(origin_point_));
}

Point QLanelet2Data::calcLlaFromEnu(const Point &point)
{
  Point lla;

  GpsTran gps_tran(origin_point_.lon, origin_point_.lat, origin_point_.ele);

  GpsDataType gps;
  NedDataType ned;

  ned.y_east = point.x;
  ned.x_north = point.y;
  ned.z_down = - point.z;

  gps_tran.fromNedToGps(gps, ned);

  lla.x = gps.longitude;
  lla.y = gps.latitude;
  lla.z = gps.altitude;

  return lla;
}
