#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "waypointmap.h"

using std::vector;

enum class VehicleState
{
  kKeepLane = 0,
  kLaneChangeLeft,
  kLaneChangeRight,
  kPrepareLaneChangeLeft,
  kPrepareLaneChangeRight,
};

class Vehicle
{
private:
  int lane_available;
  double lane_width;
  double max_velocity;
  double max_acc;
  double max_jerk;
  
  WayPointMap way_point_map;

  VehicleState state;
  double x, y;
  double s, d;
  double yaw;
  double speed;
  double acc;

  double target_speed;

public:
  Vehicle();
  Vehicle(WayPointMap &way_point_map);
  virtual ~Vehicle();

  vector<VehicleState> successor_states();

  void update_position(double x, double y,
                       double s, double d,
                       double yaw, double speed);
  void compute_path(const vector<double> &previous_path_x,
                    const vector<double> &previous_path_y,
                    vector<double> &path_x,
                    vector<double> &path_y);
};

#endif // VEHICLE_H