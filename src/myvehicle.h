#ifndef MY_VEHICLE_H
#define MY_VEHICLE_H

#include <vector>
#include <map>
#include "vehicle.h"
#include "waypointmap.h"

using std::vector;
using std::map;

/**
 * Vehicle state.
 */
enum class VehicleState
{
  kKeepLane = 0,
  kLaneChangeLeft,
  kLaneChangeRight,
  kPrepareLaneChangeLeft,
  kPrepareLaneChangeRight,
};

/**
 * Class to store my vehicle data.
 */
class MyVehicle : public Vehicle
{
private:
  // Number of available lanes
  int lane_available;
  // Maximum velocity
  double max_velocity;
  // Maximum acceleration
  double max_acc;

  // Waypoint map data
  WayPointMap way_point_map;

  // My vehicle state
  VehicleState state;
  // All other vehicles
  map<int, Vehicle> other_vehicles;

  // Target lane
  int target_lane;
  // Target acceleration
  double target_acc;
  // Target velocity
  double target_velocity;

  // Last state
  VehicleState last_state;
  // Keep track of the s distance in last change lane state.
  double keep_land_s;

  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   */
  vector<double> JMT(vector<double> &start, vector<double> &end, double T);
public:
  /**
   * Constructor.
   */
  MyVehicle();

  /**
   * Constructor with parameters.
   *
   * @param way_point_map - Waypoint data.
   */
  MyVehicle(WayPointMap &way_point_map);

  /**
   * Destructor.
   */
  virtual ~MyVehicle();

  /**
   * Update the position of the vehicle given the information from the simulator.
   *
   * @param x - Global map x coordinate of the vehicle.
   * @param y - Global map y coordinate of the vehicle.
   * @param s - Frenet s coordinate of the vehicle.
   * @param d - Frenet d coordinate of the vehicle.
   * @param yaw - Orientation of the vehicle.
   * @param v - Velocity of the vehicle.
   * @param sensor_fusion - Sensor fusion contains all the information about other vehicles.
   */
  void update_position(
    double x, double y,
    double s, double d,
    double yaw, double v,
    const vector<vector<double> > &sensor_fusion);

  /**
   * Update the state under the current situation.
   */
  void update_state();

  /**
   * Update the action under the current situation.
   */
  void update_action();

  /**
   * Compute the target path under the current situation 
   * that the car will visit sequentially every .02 seconds.
   * 
   * @param previous_path_x - Prevoius path sent back from the simulator in X coordinate.
   * @param previous_path_y - Prevoius path sent back from the simulator in Y coordinate.
   * @param path_x - (Output) Computed path in X coordinate.
   * @param path_y - (Output) Computed path in Y coordinate.
   */
  void compute_trajectory(
    const vector<double> &previous_path_x,
    const vector<double> &previous_path_y,
    vector<double> &path_x,
    vector<double> &path_y);

  /**
   * Find the vehicle just ahead.
   * 
   * @param vehicle - Vehicle ahead.
   *
   * @output if there is vehicle ahead.
   */
  bool get_vehicle_ahead(Vehicle &vehicle);

  /**
   * Find the vehicle just behind.
   * 
   * @param vehicle - Vehicle behind.
   *
   * @output if there is vehicle behind.
   */
  bool get_vehicle_behind(Vehicle &vehicle);

  /**
   * Find all the vehicles on the left.
   * 
   * @param vehicles - (Output) The found vehicles on the left.
   *
   * @output if there are vehicles on the left.
   */
  bool get_left_vehicles(vector<Vehicle> &vehicles);

  /**
   * Find all the vehicles on the right.
   * 
   * @param vehicles - (Output) The found vehicles on the right.
   *
   * @output if there are vehicles on the right.
   */
  bool get_right_vehicles(vector<Vehicle> &vehicles);

  /**
   * Check if the vehicle can change lane given the vehicles on wanted lane.
   * 
   * @param vehicles - The vehicles on wanted lane.
   *
   * @output if the vehicle can change lane given the vehicles on wanted lane.
   */
  bool can_change_lane(vector<Vehicle> &vehicles);

  /**
   * Find all the vehicles ahead.
   * 
   * @param vehicles - (Output) The found vehicles ahead.
   *
   * @output if there are vehicles ahead.
   */
  bool get_vehicles_ahead(vector<Vehicle> &vehicles);

  /**
   * Compute the optimal lane under the current situation.
   *
   * @output the optimal lane.
   */
  int get_optimal_lane();

  /**
   * Compute the target velocity under the current situation.
   *
   * @output the target velocity.
   */
  double compute_curr_velocity();

  /**
   * Dump the all the current info the object to the terminal.
   * 
   * @param left_vehicles - Vehicles on the left.
   * @param right_vehicles - Vehicles on the right.
   */
  void dump(vector<Vehicle> &left_vehicles, vector<Vehicle> &right_vehicles);
};

#endif // MY_VEHICLE_H