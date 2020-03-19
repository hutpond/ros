#ifndef GLOBALDEFINE_H
#define GLOBALDEFINE_H

#include <vector>
#include <map>
#include <cmath>
#include <QLineF>

struct Item
{
  int32_t id;
};

struct Point : public Item
{
  double x;
  double y;
  double z;

  Point & operator = (const Point &point) {
    this->id = point.id;
    this->x = point.x;
    this->y = point.y;
    this->z = point.z;

    return *this;
  }
};

struct Road : public Item
{
  enum {
    ALL = -1,
    LEFT,
    RIGHT,
    CENTRAL,
    OUTLINE
  };
  std::vector<Point> left_side;
  std::vector<Point> right_side;
  std::vector<Point> reference;

  Road & operator = (const Road &road) {
    this->id = road.id;
    this->left_side.clear();
    for (auto &point : road.left_side) {
      this->left_side.push_back(point);
    }
    this->right_side.clear();
    for (auto &point : road.right_side) {
      this->right_side.push_back(point);
    }
    this->reference.clear();
    for (auto &point : road.reference) {
      this->reference.push_back(point);
    }

    return *this;
  }
};

struct RoadSegment : public Item
{
  enum {
    ROAD,
    SQUARE
  };
  int type;
  int central_start_index;
  int central_end_index;
  std::map<int, Road> roads;

  RoadSegment & operator = (const RoadSegment &segment) {
    this->id = segment.id;
    this->type = segment.type;
    this->roads.clear();
    for (const auto &road : segment.roads) {
      this->roads[road.first] = road.second;
    }
  }
};

struct TrafficLight : public Item
{
  enum {
    UNKNOWN_SIGN,
    STOP_SIGN,
    MAX_SPEED_SIGN,
    MIN_SPEED_SIGN
  };

  int type;
  std::vector<Point> points;

  TrafficLight & operator = (const TrafficLight &light) {
    this->id = light.id;
    this->type = light.type;
    this->points = light.points;
  }
};

struct Crossing : public Item
{
  std::vector<Point> points;

  Crossing & operator = (const Crossing &crossing) {
    this->id = crossing.id;
    this->points = crossing.points;
  }
};

struct Marking : public Item
{
  enum MARKING_TYPE {
    UNKNOWN_MARK,
    TEXT_MARK,
    AF_MARK,
    AL_MARK,
    AR_MARK,
    AFL_MARK,
    AFR_MARK,
    ALR_MARK,
    UTURN_MARK,
    NOUTURN_MARK
  };
  int type;
  Point point;

  Marking & operator = (const Marking &marking) {
    this->id = marking.id;
    this->type = marking.type;
    this->point = marking.point;
  }
};

struct HdMapRaw : public Item
{
  std::vector<RoadSegment> road_segments;
  std::vector<TrafficLight> traffic_lights;
  std::vector<Point> stop_lines;
//  std::vector<Curb> curbs;
//  std::vector<Boundary> boundaries;
  std::vector<Crossing> crossings;
  std::vector<Marking> markings;
  std::vector<Point> signs;

  int32_t id_selected{-1};

  HdMapRaw & operator = (const HdMapRaw &hdmap) {
    this->id = hdmap.id;
    this->id_selected = hdmap.id_selected;

    this->road_segments.clear();
    for (const auto &segment : hdmap.road_segments) {
      this->road_segments.push_back(segment);
    }

    this->traffic_lights.clear();
    for (const auto &light : hdmap.traffic_lights) {
      this->traffic_lights.push_back(light);
    }

    this->stop_lines.clear();
    for (const auto &point : hdmap.stop_lines) {
      this->stop_lines.push_back(point);
    }

    this->crossings.clear();
    for (const auto &point : hdmap.crossings) {
      this->crossings.push_back(point);
    }

    this->markings.clear();
    for (const auto &marking : hdmap.markings) {
      this->markings.push_back(marking);
    }

    this->signs.clear();
    for (const auto &point : hdmap.signs) {
      this->signs.push_back(point);
    }
  }
};

struct MapPoint
{
  int id;

  double lat;
  double lon;
  double alt;

  double east;
  double north;
  double up;

  double pitch;
  double roll;
  double yaw;

  double left_w;
  double right_w;

  double curvature;
  double slope;
  int dir;   // -1: left  0: line 1: right
  int side;  // -1: left  1: right
  int bush;  // 0: has not 1: has
  bool insert;
  bool select;

  MapPoint()
    : lat(0.0), lon(0.0), alt(0.0)
    , east(0.0), north(0.0), up(0.0)
    , pitch(0.0), roll(0.0), yaw(0.0)
    , left_w(0.0), right_w(0.0)
    , curvature(0.0), slope(0.0)
    , dir(1), side(1), bush(false)
    , insert(false), select(false)
  {}

  double distance(const MapPoint &point) {
    return std::sqrt(
          std::pow(this->east - point.east, 2) +
          std::pow(this->north - point.north, 2) +
          std::pow(this->up - point.up, 2)
          );
  }

  MapPoint & operator=(const MapPoint &rh) {
    this->id = rh.id;

    this->lat = rh.lat;
    this->lon = rh.lon;
    this->alt = rh.alt;

    this->east = rh.east;
    this->north = rh.north;
    this->up = rh.up;

    this->pitch = rh.pitch;
    this->roll = rh.roll;
    this->yaw = rh.yaw;

    this->left_w = rh.left_w;
    this->right_w = rh.right_w;

    this->curvature = rh.curvature;
    this->dir = rh.dir;
    this->side = rh.side;
    this->bush = rh.bush;

    return *this;
  }

  double angle() {
    QLineF line(0, 0, 0, 10);
    QLineF line2(0, 0, this->east, this->north);
    return line.angleTo(line2);
  }
};



#endif // GLOBALDEFINE_H
