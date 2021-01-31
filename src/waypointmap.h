#ifndef WAYPOINTMAP_H
#define WAYPOINTMAP_H

#include <vector>

using std::vector;

class WayPointMap {

public:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

public:
  WayPointMap();
  virtual ~WayPointMap();

  void Add(double x, double y, double s, double dx, double dy);
};

#endif // WAYPOINTMAP_H
