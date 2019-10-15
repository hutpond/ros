#ifndef MAPDEFINES_H
#define MAPDEFINES_H

enum class LaneSide
{
  Central,
  Left,
  Right
};

enum class LaneDirection
{
  Forward,
  Reverse
};

enum BoundarySide
{
  Left,
  Right
};

enum class MapOperation
{
  None,
  Boundary,
  SignalSign,
  Crosswalk,
  StopSign,
  YieldSign,
  ClearArea,
  SpeedBump
};

#endif // MAPDEFINES_H
