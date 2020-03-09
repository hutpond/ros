#ifndef GLOBALDEFINE_H
#define GLOBALDEFINE_H

#include <vector>
#include <map>

struct Point
{
  double x;
  double y;
  double z;

  Point & operator = (const Point &point) {
    this->x = point.x;
    this->y = point.y;
    this->z = point.z;

    return *this;
  }
};

struct Road
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

struct RoadSegment
{
  enum {
    ROAD,
    SQUARE
  };
  int type;
  std::map<int, Road> roads;

  RoadSegment & operator = (const RoadSegment &segment) {
    this->type = segment.type;
    this->roads.clear();
    for (const auto &road : segment.roads) {
      this->roads[road.first] = road.second;
    }
  }
};

struct TrafficLight
{
  enum {
    UNKNOWN_SIGN,
    STOP_SIGN,
    MAX_SPEED_SIGN,
    MIN_SPEED_SIGN
  };

  int type;
  Point point;

  TrafficLight & operator = (const TrafficLight &light) {
    this->type = light.type;
    this->point = light.point;
  }
};

struct Marking
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
    this->type = marking.type;
    this->point = marking.point;
  }
};

struct HdMap
{
  std::vector<RoadSegment> road_segments;
  std::vector<TrafficLight> traffic_lights;
  std::vector<Point> stop_lines;
//  std::vector<Curb> curbs;
//  std::vector<Boundary> boundaries;
  std::vector<Point> crossings;
  std::vector<Marking> markings;
  std::vector<Point> signs;

  HdMap & operator = (const HdMap &hdmap) {
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


#endif // GLOBALDEFINE_H
