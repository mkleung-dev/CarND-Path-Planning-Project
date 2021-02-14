#include "waypointmap.h"

WayPointMap::WayPointMap() {
  /**
   * Constructor
   */ 
}

WayPointMap::~WayPointMap() {
  /**
   * Destructor
   */
}

void WayPointMap::Add(double x, double y, double s, double dx, double dy) {
  /**
   * Add the data to the object.
   * @param x - Global map position x.
   * @param y - Global map position y.
   * @param s - Corresponding Frenet s value.
   * @param dx - Corresponding Frenet d normal vector in x component.
   * @param dy - Corresponding Frenet d normal vector in y component.
   */
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(dx);
    map_waypoints_dy.push_back(dy);
}