#include "waypointmap.h"

WayPointMap::WayPointMap() {
  
}

WayPointMap::~WayPointMap() {
  
}

void WayPointMap::Add(double x, double y, double s, double dx, double dy) {
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(dx);
    map_waypoints_dy.push_back(dy);
}