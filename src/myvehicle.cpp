#include "myvehicle.h"

#include <iostream>
#include "helpers.h"
#include "spline.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using std::vector;
using std::map;
using std::cout;
using std::endl;

using Eigen::VectorXd;
using Eigen::MatrixXd;

#define CHANGE_LANE_DIST 70.0

MyVehicle::MyVehicle() : Vehicle() {
  /**
   * Constructor.
   */
  lane_available = 3;
  max_velocity = 48.5 / 2.24;
  max_acc = 8;
  target_velocity = max_velocity;
  this->target_lane = 1;
}

MyVehicle::MyVehicle(WayPointMap &way_point_map) : MyVehicle() {
  /**
   * Constructor with parameters.
   *
   * @param way_point_map - Waypoint data.
   */
  this->way_point_map = way_point_map;
  this->target_lane = 1;
}

MyVehicle::~MyVehicle() {
  /**
   * Destructor.
   */
}

void MyVehicle::update_position(
  double x, double y,
  double s, double d,
  double yaw, double v,
  const vector<vector<double> > &sensor_fusion) {
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
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->v = v;

  other_vehicles.clear();
  for (int i = 0; i < sensor_fusion.size(); i++) {
    int sensor_fusion_id = sensor_fusion[i][0];
    double sensor_fusion_x = sensor_fusion[i][1];
    double sensor_fusion_y = sensor_fusion[i][2];
    double sensor_fusion_vx = sensor_fusion[i][3];
    double sensor_fusion_vy = sensor_fusion[i][4];
    double sensor_fusion_s = sensor_fusion[i][5];
    double sensor_fusion_d = sensor_fusion[i][6];
    other_vehicles[sensor_fusion_id] = Vehicle(sensor_fusion_id, sensor_fusion_x, sensor_fusion_y,
                                               sensor_fusion_s, sensor_fusion_d,
                                               atan2(sensor_fusion_vy, sensor_fusion_vx),
                                               sqrt(sensor_fusion_vx * sensor_fusion_vx + sensor_fusion_vy * sensor_fusion_vy));
  }
}

void MyVehicle::update_state() {
  /**
   * Update the state under the current situation.
   */
  Vehicle vehicle;

  vector<Vehicle> left_vehicles;
  vector<Vehicle> right_vehicles;

  Vehicle zero_vehicle;
  if (this->get_s_diff(zero_vehicle, -(keep_land_s + CHANGE_LANE_DIST)) > 0) {
    keep_land_s = this->get_s() - CHANGE_LANE_DIST - 1;
  }

  if (state == VehicleState::kKeepLane) {
    int optimal_lane = get_optimal_lane();
    // cout << "self_lane: " << this->get_lane() << " max_lane: " << optimal_lane << endl;
    if (this->get_s_diff(zero_vehicle, -(keep_land_s + CHANGE_LANE_DIST)) > 0) {
      // cout << "can change lane." << endl;
      if (optimal_lane < this->target_lane) {
        state = VehicleState::kPrepareLaneChangeLeft;
      } else if (optimal_lane > this->target_lane) {
        state = VehicleState::kPrepareLaneChangeRight;
      }
    }
  } else if (state == VehicleState::kPrepareLaneChangeLeft) {
    int optimal_lane = get_optimal_lane();
    // cout << "self_lane: " << this->get_lane() << " max_lane: " << max_speed_lane << endl;
    Vehicle zero_vehicle;
    if (optimal_lane == this->target_lane) {
      state = VehicleState::kKeepLane;
    } else if (optimal_lane > this->target_lane) {
      state = VehicleState::kPrepareLaneChangeRight;
    } else {
      bool change = true;
      if (get_left_vehicles(left_vehicles)) {
        change = can_change_lane(left_vehicles);
      }
      if (fabs(get_velocity() - target_velocity) > 3) {
        change = false;
      }
      if (change) {
        state = VehicleState::kLaneChangeLeft;
      }
    }
  } else if (state == VehicleState::kPrepareLaneChangeRight) {
    int optimal_lane = get_optimal_lane();
    // cout << "self_lane: " << this->get_lane() << " max_lane: " << max_speed_lane << endl;
    Vehicle zero_vehicle;
    if (optimal_lane == this->target_lane) {
      state = VehicleState::kKeepLane;
    } else if (optimal_lane < this->target_lane) {
      state = VehicleState::kPrepareLaneChangeLeft;
    } else {
      bool change = true;
      if (get_right_vehicles(right_vehicles)) {
        change = can_change_lane(right_vehicles);
      }
      if (fabs(get_velocity() - target_velocity) > 3) {
        change = false;
      }
      if (change) {
        state = VehicleState::kLaneChangeRight;
      }
    }
  } else if (state == VehicleState::kLaneChangeLeft) {
    state = VehicleState::kKeepLane;
    keep_land_s = this->get_s();
  } else if (state == VehicleState::kLaneChangeRight) {
    state = VehicleState::kKeepLane;
    keep_land_s = this->get_s();
  }
  if (last_state != state) {
    dump(left_vehicles, right_vehicles);
  }
  last_state = state;
}

void MyVehicle::update_action() {
  /**
   * Update the action under the current situation.
   */
  target_acc = max_acc;
  if (state == VehicleState::kKeepLane) {
    Vehicle zero_vehicle;
    if (this->get_s_diff(zero_vehicle, -(keep_land_s + CHANGE_LANE_DIST)) < 0) {
      target_acc = max_acc  / 2;
    }
    target_velocity = compute_curr_velocity();
  } else if (state == VehicleState::kLaneChangeLeft) {
    this->target_lane = this->target_lane - 1;
  } else if (state == VehicleState::kLaneChangeRight) {
    this->target_lane = this->target_lane + 1;
  } else if (state == VehicleState::kPrepareLaneChangeLeft) {
    target_velocity = compute_curr_velocity();
  } else if (state == VehicleState::kPrepareLaneChangeRight) {
    target_velocity = compute_curr_velocity();
  }
}

void MyVehicle::compute_trajectory(
  const vector<double> &previous_path_x, const vector<double> &previous_path_y,
  vector<double> &path_x, vector<double> &path_y) {
  /**
   * Compute the target path under the current situation 
   * that the car will visit sequentially every .02 seconds.
   * 
   * @param previous_path_x - Prevoius path sent back from the simulator in X coordinate.
   * @param previous_path_y - Prevoius path sent back from the simulator in Y coordinate.
   * @param path_x - (Output) Computed path in X coordinate.
   * @param path_y - (Output) Computed path in Y coordinate.
   */
  
  vector<double> x_for_spline;
  vector<double> y_for_spline;
  double ref_x = 0, ref_y = 0, ref_yaw = 0;
  double ref_x_prev = 0, ref_y_prev = 0;
  double ref_velocity = 0, ref_acc = 0;
  double ref_velocity_prev = 0;
  int prev_size = previous_path_x.size();

  // Add the previous path.
  for (int i = 0; i < prev_size; i++)
  {
    path_x.push_back(previous_path_x[i]);
    path_y.push_back(previous_path_y[i]);

    x_for_spline.push_back(previous_path_x[i]);
    y_for_spline.push_back(previous_path_y[i]);
  }

  // Compute the future path.

  // Estimate the variable to compute the future path.
  if (prev_size < 2)
  {
    ref_x = x;
    ref_y = y;

    ref_x_prev = ref_x - cos(yaw);
    ref_y_prev = ref_y - sin(yaw);

    ref_velocity = v;
    ref_velocity_prev = v;

    keep_land_s = this->get_s();
    target_lane = this->get_lane();
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    ref_x_prev = previous_path_x[prev_size - 2];
    ref_y_prev = previous_path_y[prev_size - 2];

    ref_velocity = sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / 0.02;
    ref_velocity_prev = v;

    if (previous_path_x.size() > 2) {
      double ref_x_prev_prev = previous_path_x[prev_size - 3];
      double ref_y_prev_prev = previous_path_y[prev_size - 3];

      ref_velocity_prev = sqrt((ref_x_prev - ref_x_prev_prev) * (ref_x_prev - ref_x_prev_prev) + (ref_y_prev - ref_y_prev_prev) * (ref_y_prev - ref_y_prev_prev)) / 0.02;
      ref_acc = (ref_velocity - ref_velocity_prev) / 0.02;
    }
  }

  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

  vector<double> tempSD = getFrenet(ref_x, ref_y, ref_yaw, way_point_map);
  if (prev_size == 0) {
    x_for_spline.push_back(ref_x_prev);
    y_for_spline.push_back(ref_y_prev);
    x_for_spline.push_back(ref_x);
    y_for_spline.push_back(ref_y);
  }

  double start_change = ref_velocity * 1.5;
  if (start_change < 10) {
    start_change = 10;
  }
  for (double step = start_change; step < start_change + 30.0 * 2 + 1; step += 30.0) {
    vector<double> tempXY = getXY(tempSD[0] + step, (2.0 + 4.0 * target_lane), way_point_map);
    x_for_spline.push_back(tempXY[0]);
    y_for_spline.push_back(tempXY[1]);
  }

  // Transformat the coordinates so it is reference from my vehicle.
  for (int i = 0; i < x_for_spline.size(); i++) {
    double t_x = x_for_spline[i] - ref_x;
    double t_y = y_for_spline[i] - ref_y;

    x_for_spline[i] = (t_x * cos(-ref_yaw) - t_y * sin(-ref_yaw));
    y_for_spline[i] = (t_x * sin(-ref_yaw) + t_y * cos(-ref_yaw));
  }

  tk::spline s;
  s.set_points(x_for_spline, y_for_spline);

  double x_add_on = 0;
  double y_add_on = 0;

  double real_time_velocity = ref_velocity;
  for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
    if (real_time_velocity < target_velocity) {
      real_time_velocity += target_acc * 0.02;
      if (real_time_velocity > target_velocity) {
        real_time_velocity = target_velocity;
      }
    } else if (real_time_velocity > target_velocity) {
      real_time_velocity -= target_acc * 0.02;
      if (real_time_velocity < target_velocity) {
        real_time_velocity = target_velocity;
      }
    }

    // compute the x, y which have the exact velocity
    double expected_diff = 0.02 * real_time_velocity;
    double target_x = x_add_on + expected_diff;
    double target_y = s(target_x);
    double diff_x = target_x - x_add_on;
    double diff_y = target_y - y_add_on;
    double base = sqrt(diff_x * diff_x + diff_y * diff_y);
    double ratio = expected_diff / base;
    do {
      target_x = x_add_on + expected_diff * ratio;
      target_y = s(target_x);
      diff_x = target_x - x_add_on;
      diff_y = target_y - y_add_on;
      base = sqrt(diff_x * diff_x + diff_y * diff_y);
      ratio = expected_diff / base;
    } while (fabs(expected_diff - base) > 0.0001);

    double x_point = target_x;
    double y_point = s(x_point);
    x_add_on = x_point;
    y_add_on = y_point;

    double x_point_car_coor = x_point;
    double y_point_car_coor = y_point;

    //Compute back to gloabl XY coordinates.
    x_point = ref_x + (x_point_car_coor * cos(ref_yaw) - y_point_car_coor * sin(ref_yaw));
    y_point = ref_y + (x_point_car_coor * sin(ref_yaw) + y_point_car_coor * cos(ref_yaw));

    path_x.push_back(x_point);
    path_y.push_back(y_point);
  }

}

bool MyVehicle::get_vehicle_ahead(Vehicle &vehicle) {
  /**
   * Find the vehicle just ahead.
   * 
   * @param vehicle - Vehicle ahead.
   *
   * @output if there is vehicle ahead.
   */
  bool vehicle_found = false;
  bool first = true;
  double min_s = -1;
  for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
    Vehicle currVehicle = it->second;
    double s = currVehicle.get_s_diff(*this, 0);
    if (fabs(currVehicle.get_d() - this->get_d()) < 3.0 && s > 0) {
      if (first || s < min_s) {
        first = false;
        min_s = s;
        vehicle = currVehicle;
        vehicle_found = true;
      }
    }
  }
  return vehicle_found;
}

bool MyVehicle::get_vehicle_behind(Vehicle &vehicle) {
  /**
   * Find the vehicle just behind.
   * 
   * @param vehicle - Vehicle behind.
   *
   * @output if there is vehicle behind.
   */
  bool vehicle_found = false;
  bool first = true;
  double max_s = -1;
  for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
    Vehicle currVehicle = it->second;
    double s = currVehicle.get_s_diff(*this, 0);
    if (fabs(currVehicle.get_d() - this->get_d()) < 3.0 && s < 0) {
      if (first || s > max_s) {
        first = false;
        max_s = s;
        vehicle = currVehicle;
        vehicle_found = true;
      }
    }
  }
  return vehicle_found;
}

bool MyVehicle::get_left_vehicles(vector<Vehicle> &vehicles) {
  /**
   * Find all the vehicles on the left.
   * 
   * @param vehicles - (Output) The found vehicles on the left.
   *
   * @output if there are vehicles on the left.
   */
  bool vehicle_found = false;
  vehicles.clear();
  if (this->get_lane() > 0) {
    for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
      Vehicle vehicle = it->second;
      if (fabs(vehicle.get_d() - (2 + 4 * (this->get_lane() - 1))) < 3.0) {
        vehicles.push_back(it->second);
        vehicle_found = true;
      }
    }
  }
  return vehicle_found;
}
bool MyVehicle::get_right_vehicles(vector<Vehicle> &vehicles) {
  /**
   * Find all the vehicles on the right.
   * 
   * @param vehicles - (Output) The found vehicles on the right.
   *
   * @output if there are vehicles on the right.
   */
  bool vehicle_found = false;
  vehicles.clear();
  if (this->get_lane() < lane_available - 1) {
    for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
      Vehicle vehicle = it->second;
      if (fabs(vehicle.get_d() - (2 + 4 * (this->get_lane() + 1))) < 3) {
        vehicles.push_back(vehicle);
        vehicle_found = true;
      }
    }
  }
  return vehicle_found;
}

bool MyVehicle::can_change_lane(vector<Vehicle> &vehicles) {
  /**
   * Check if the vehicle can change lane given the vehicles on wanted lane.
   * 
   * @param vehicles - The vehicles on wanted lane.
   *
   * @output if the vehicle can change lane given the vehicles on wanted lane.
   */
  for (int i = 0; i < vehicles.size(); i++) {
    double behind_speed_diff = vehicles[i].get_velocity() - this->get_velocity();
    double ahead_speed_diff = this->get_velocity() - vehicles[i].get_velocity();
    if (behind_speed_diff < 0) {
      behind_speed_diff = 0;
    }
    if (ahead_speed_diff < 0) {
      ahead_speed_diff = 0;
    }
    if (this->get_s_diff(vehicles[i], -10 - vehicles[i].get_velocity() * 1.0) < 0 &&  //car behind
        this->get_s_diff(vehicles[i], 10 + this->get_velocity() * 1.0) > 0) //car ahead
    {
      return false;
    }
  }
  return true;
}

bool MyVehicle::get_vehicles_ahead(vector<Vehicle> &vehicles) {
  /**
   * Find all the vehicles ahead.
   * 
   * @param vehicles - (Output) The found vehicles ahead.
   *
   * @output if there are vehicles ahead.
   */
  bool vehicle_found = false;
  vehicles.resize(lane_available);
  for (int i = 0; i < vehicles.size(); i++) {
    vehicles[i] = Vehicle();
  }
  for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
    Vehicle vehicle = it->second;
    int lane = vehicle.get_lane();
    if (0 <= lane && lane < vehicles.size()) {
      if (vehicle.get_s_diff(*this, 0) > 0) {
        if (vehicles[lane].get_id() == -1 || vehicle.get_s_diff(vehicles[lane], 0) < 0) {
          vehicles[lane] = vehicle;
          vehicle_found = true;
        }
      }
    }
  }
  return vehicle_found;
}

int MyVehicle::get_optimal_lane() {
  /**
   * Compute the optimal lane under the current situation.
   *
   * @output the optimal lane.
   */
  vector<Vehicle> vehicles;
  get_vehicles_ahead(vehicles);

  int optimal_lane = this->target_lane;
  Vehicle max_vehicle = vehicles[optimal_lane];

  if (max_vehicle.get_id() != -1) {
    for (int i = 0; i < vehicles.size(); i++) {
      if (vehicles[i].get_id() == -1) {
        if (max_vehicle.get_id() == -1) {
          if (abs(this->target_lane - vehicles[i].get_lane()) < abs(this->target_lane - max_vehicle.get_lane())) {
            max_vehicle = vehicles[i];
            optimal_lane = i;
          }
        } else {
          max_vehicle = vehicles[i];
          optimal_lane = i;
        }
      } else if (max_vehicle.get_id() != -1) {
        double dist = vehicles[i].get_s_diff(max_vehicle, 0);
        if (dist > 0) {
          max_vehicle = vehicles[i];
          optimal_lane = i;
        }
      }
    }
  }
  return optimal_lane;
}

double MyVehicle::compute_curr_velocity() {
  /**
   * Compute the target velocity under the current situation.
   *
   * @output the target velocity.
   */
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
  double target_speed = max_velocity;
  double speed_ahead = target_speed;
  double speed_behind = 0;
  if (get_vehicle_ahead(vehicle_ahead)) {
    // cout << "get_vehicle_ahead," << vehicle_ahead.get_speed() << endl;
    double ahead_speed_diff = vehicle_ahead.get_velocity() - this->get_velocity();
    if (vehicle_ahead.get_s_diff(*this, -15 - this->get_velocity() * 1.0) < 0) {
      speed_ahead = vehicle_ahead.get_velocity() - 2.0;
    }
    if (vehicle_ahead.get_s_diff(*this, -this->get_velocity() * 1.0) < 0) {
      speed_ahead = this->get_velocity() / 2.0;
    }
    if (vehicle_ahead.get_s_diff(*this, -10) < 0) {
      speed_ahead = 0;
    }
  }
  target_speed = speed_ahead;

  return target_speed;
}

void MyVehicle::dump(vector<Vehicle> &left_vehicles, vector<Vehicle> &right_vehicles) {
  /**
   * Dump the all the current info the object to the terminal.
   * 
   * @param left_vehicles - Vehicles on the left.
   * @param right_vehicles - Vehicles on the right.
   */
  cout << "************************" << endl;
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%H,%M,%S", &tstruct);

  cout << "Time," << buf << endl;
  cout << "My vehicle" << endl;
  cout << "ID," << get_id();
  cout << ",X," << get_x();
  cout << ",Y," << get_y();
  cout << ",S," << get_s();
  cout << ",D," << get_d();
  cout << ",Speed," << get_velocity();
  cout << ",Yaw," << get_yaw();
  cout << ",Lane," << get_lane();
  cout << endl;
  cout << "keep_land_s," << keep_land_s << endl;

  if (state == VehicleState::kKeepLane) {
    cout << "VehicleState::kKeepLane";
  } else if (state == VehicleState::kLaneChangeLeft) {
    cout << "VehicleState::kLaneChangeLeft";
  } else if (state == VehicleState::kLaneChangeRight) {
    cout << "VehicleState::kLaneChangeRight";
  } else if (state == VehicleState::kPrepareLaneChangeLeft) {
    cout << "VehicleState::kPrepareLaneChangeLeft";
  } else if (state == VehicleState::kPrepareLaneChangeRight) {
    cout << "VehicleState::kPrepareLaneChangeRight";
  }
  cout << endl;

  cout << "Other vehicles" << endl;
  for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
    cout << "ID," << it->second.get_id();
    cout << ",X," << it->second.get_x();
    cout << ",Y," << it->second.get_y();
    cout << ",S," << it->second.get_s();
    cout << ",D," << it->second.get_d();
    cout << ",Speed," << it->second.get_velocity();
    cout << ",Yaw," << it->second.get_yaw();
    cout << ",Lane," << it->second.get_lane();
    cout << endl;
  }

  cout << "Left vehicles" << endl;
  for (int i = 0; i < left_vehicles.size(); i++) {
    cout << "ID," << left_vehicles[i].get_id();
    cout << ",X," << left_vehicles[i].get_x();
    cout << ",Y," << left_vehicles[i].get_y();
    cout << ",S," << left_vehicles[i].get_s();
    cout << ",D," << left_vehicles[i].get_d();
    cout << ",Speed," << left_vehicles[i].get_velocity();
    cout << ",Yaw," << left_vehicles[i].get_yaw();
    cout << ",Lane," << left_vehicles[i].get_lane();
    cout << endl;
  }

  cout << "Right vehicles" << endl;
  for (int i = 0; i < right_vehicles.size(); i++) {
    cout << "ID," << right_vehicles[i].get_id();
    cout << ",X," << right_vehicles[i].get_x();
    cout << ",Y," << right_vehicles[i].get_y();
    cout << ",S," << right_vehicles[i].get_s();
    cout << ",D," << right_vehicles[i].get_d();
    cout << ",Speed," << right_vehicles[i].get_velocity();
    cout << ",Yaw," << right_vehicles[i].get_yaw();
    cout << ",Lane," << right_vehicles[i].get_lane();
    cout << endl;
  }
  cout << "------------------------" << endl;
}

vector<double> MyVehicle::JMT(vector<double> &start, vector<double> &end, double T) {
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
   double alpha0 = start[0];
   double alpha1 = start[1];
   double alpha2 = start[2] / 2;
   
   double dT1 = T;
   double dT2 = dT1 * dT1;
   double dT3 = dT2 * dT1;
   double dT4 = dT2 * dT2;
   double dT5 = dT2 * dT3;
   
   MatrixXd A(3, 3);
   VectorXd b(3);
   A <<     dT3,      dT4,      dT5,
        3 * dT2,  4 * dT3,  5 * dT4,
        6 * dT1, 12 * dT2, 20 * dT3;
   b << end[0] - (start[0] + start[1] * dT1 + start[2] * dT2 / 2),
        end[1] - (start[1] + start[2] * dT1),
        end[2] - start[2];
   VectorXd sol = A.colPivHouseholderQr().solve(b);;

  return {alpha0,alpha1,alpha2,sol(0),sol(1),sol(2)};
}