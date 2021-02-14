#ifndef WAYPOINTMAP_H
#define WAYPOINTMAP_H

#include <vector>

using std::vector;

/**
 * Class to store waypoint data.
 */ 
class WayPointMap {
public:
  // Global map position x.
  vector<double> map_waypoints_x;
  // Global map position y.
  vector<double> map_waypoints_y;
  // Corresponding Frenet s value.
  vector<double> map_waypoints_s;
  // Corresponding Frenet d normal vector in x component.
  vector<double> map_waypoints_dx;
  // Corresponding Frenet d normal vector in y component.
  vector<double> map_waypoints_dy;

public:
  /**
   * Constructor
   */ 
  WayPointMap();
  /**
   * Destructor
   */
  virtual ~WayPointMap();
  
  /**
   * Add the waypoint data to the object.
   * 
   * @param x - Global map position x.
   * @param y - Global map position y.
   * @param s - Corresponding Frenet s value.
   * @param dx - Corresponding Frenet d normal vector in x component.
   * @param dy - Corresponding Frenet d normal vector in y component.
   */
  void Add(double x, double y, double s, double dx, double dy);
};

#endif // WAYPOINTMAP_H
