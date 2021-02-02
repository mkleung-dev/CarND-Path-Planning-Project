#ifndef MY_VEHICLE_H
#define MY_VEHICLE_H

#include <vector>
#include <map>
#include "vehicle.h"
#include "waypointmap.h"

using std::vector;
using std::map;

enum class VehicleState
{
  kKeepLane = 0,
  kLaneChangeLeft,
  kLaneChangeRight,
  kPrepareLaneChangeLeft,
  kPrepareLaneChangeRight,
};

class MyVehicle : public Vehicle
{
private:
  int lane_available;
  double max_velocity;
  double max_acc;
  double max_jerk;

  WayPointMap way_point_map;
  map<int, Vehicle> other_vehicles;

  VehicleState state;
  double acc;

  double target_speed;

public:
  MyVehicle();
  MyVehicle(WayPointMap &way_point_map);
  virtual ~MyVehicle();

  vector<VehicleState> successor_states();

  void update_position(double x, double y,
                       double s, double d,
                       double yaw, double speed,
                       const vector<vector<double> > &sensor_fusion);
  void update_state();
  void compute_path(const vector<double> &previous_path_x,
                    const vector<double> &previous_path_y,
                    vector<double> &path_x,
                    vector<double> &path_y);

  bool get_vehicle_ahead(Vehicle &vehicle);
  bool get_vehicle_behind(Vehicle &vehicle);
};

#endif // MY_VEHICLE_H