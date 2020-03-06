#ifndef GLOBALDEFINE_H
#define GLOBALDEFINE_H

#include <vector>
#include <map>

enum class ItemType {
  TypeSegment,
  TypeRoad,
  TypeRoadSide,
  TypePoint,
  TypeTrafficLignt
};

struct Point
{
  double x;
  double y;
  double z;
};

struct Road
{
  enum {
    LEFT,
    RIGHT,
    CENTRAL,
    OUTLINE
  };
  std::vector<Point> left_side;
  std::vector<Point> right_side;
  std::vector<Point> reference;
};

struct RoadSegment
{
  enum {
    ROAD,
    SQUARE
  };
  int type;
  std::map<int, Road> roads;
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
};


#endif // GLOBALDEFINE_H
