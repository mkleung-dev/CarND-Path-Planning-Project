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
  int target_lane;

  WayPointMap way_point_map;
  map<int, Vehicle> other_vehicles;

  VehicleState state;
  VehicleState last_state;
  double acc;

  double keep_land_s;
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
  void update_action();
  void compute_path(const vector<double> &previous_path_x,
                    const vector<double> &previous_path_y,
                    vector<double> &path_x,
                    vector<double> &path_y);

  bool get_vehicle_ahead(Vehicle &vehicle);
  bool get_vehicle_behind(Vehicle &vehicle);
  bool get_left_vehicles(vector<Vehicle> &vehicles);
  bool get_right_vehicles(vector<Vehicle> &vehicles);
  bool can_change_lane(vector<Vehicle> &vehicles);

  void get_vehicles_ahead(vector<Vehicle> &vehicles);
  int get_optimal_lane();

  double compute_curr_speed();

  void dump(vector<Vehicle> &left_vehicles, vector<Vehicle> &right_vehicles);
};

#endif // MY_VEHICLE_H