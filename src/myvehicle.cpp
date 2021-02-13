#include "myvehicle.h"

#include <iostream>
#include "helpers.h"
#include "spline.h"

using std::vector;
using std::map;
using std::cout;
using std::endl;

#define CHANGE_LANE_DIST 70.0

MyVehicle::MyVehicle() : Vehicle() {
  lane_available = 3;
  max_velocity = 48.5 / 2.24;
  max_acc = 5;
  max_jerk = 9.5;
  target_speed = max_velocity;
  this->target_lane = 1;
}

MyVehicle::MyVehicle(WayPointMap &way_point_map) : MyVehicle() {
  this->way_point_map = way_point_map;
  this->target_lane = 1;
}

MyVehicle::~MyVehicle() {

}

vector<VehicleState> MyVehicle::successor_states() {
  vector<VehicleState> states;
  states.push_back(VehicleState::kKeepLane);
  int lane = (int)this->d / lane_width;
  if (this->state == VehicleState::kKeepLane) {
    if (lane != lane_available - 1) {
      states.push_back(VehicleState::kPrepareLaneChangeLeft);
    } else {
      states.push_back(VehicleState::kPrepareLaneChangeRight);
    }
  } else if (this->state == VehicleState::kPrepareLaneChangeLeft) {
    if (lane != lane_available - 1) {
      states.push_back(VehicleState::kPrepareLaneChangeLeft);
      states.push_back(VehicleState::kLaneChangeLeft);
    }
  } else if (this->state == VehicleState::kPrepareLaneChangeRight) {
    if (lane != 0) {
      states.push_back(VehicleState::kPrepareLaneChangeRight);
      states.push_back(VehicleState::kLaneChangeRight);
    }
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

void MyVehicle::update_position(double x, double y,
                              double s, double d,
                              double yaw, double speed,
                              const vector<vector<double> > &sensor_fusion)
{
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;

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
  // std::cout << "other_vehicles,size," << other_vehicles.size() << std::endl;
  // std::cout << "update_position,x," << x << ",y," << y << ",s," << s << ",d," << d << ",yaw," << yaw << ",speed," << speed << std::endl;
}

bool MyVehicle::can_change_lane(vector<Vehicle> &vehicles) {
  for (int i = 0; i < vehicles.size(); i++) {
    double behind_speed_diff = vehicles[i].get_speed() - this->get_speed();
    double ahead_speed_diff = this->get_speed() - vehicles[i].get_speed();
    if (behind_speed_diff < 0) {
      behind_speed_diff = 0;
    }
    if (ahead_speed_diff < 0) {
      ahead_speed_diff = 0;
    }
    if (this->get_s_diff(vehicles[i], -10 - vehicles[i].get_speed() * 1.0) < 0 &&  //car behind
        this->get_s_diff(vehicles[i], 10 + this->get_speed() * 1.0) > 0) //car ahead
    {
      return false;
    }
  }
  return true;
}

void MyVehicle::update_state() {
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
      if (fabs(get_speed() - target_speed) > 3) {
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
      if (fabs(get_speed() - target_speed) > 3) {
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
  // if (last_state != state) {
    dump(left_vehicles, right_vehicles);
  // }
  last_state = state;
}

double MyVehicle::compute_curr_speed() {
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
  double target_speed = max_velocity;
  double speed_ahead = target_speed;
  double speed_behind = 0;
  if (get_vehicle_ahead(vehicle_ahead)) {
    // cout << "get_vehicle_ahead," << vehicle_ahead.get_speed() << endl;
    double ahead_speed_diff = vehicle_ahead.get_speed() - this->get_speed();
    if (vehicle_ahead.get_s_diff(*this, -15 - this->get_speed() * 1.0) < 0) {
      speed_ahead = vehicle_ahead.get_speed() - 2.0;
    }
    if (vehicle_ahead.get_s_diff(*this, -this->get_speed() * 1.0) < 0) {
      speed_ahead = this->get_speed() / 4.0;
    }
    if (vehicle_ahead.get_s_diff(*this, -10) < 0) {
      speed_ahead = 0;
    }
  }
  // if (get_vehicle_behind(vehicle_behind)) {
  //   if (this->get_s_diff(vehicle_behind, -5 - vehicle_behind.get_speed() * 1.0) < 0) {
  //     speed_behind = vehicle_behind.get_speed();
  //   }
  // }
  // if (speed_ahead > speed_behind) {
  //   target_speed = speed_ahead;
  // } else {
  //   target_speed = (speed_ahead + speed_behind) / 2;
  // }
  target_speed = speed_ahead;

  return target_speed;
}

void MyVehicle::update_action() {
  acc = max_acc;
  if (state == VehicleState::kKeepLane) {
    Vehicle zero_vehicle;
    if (this->get_s_diff(zero_vehicle, -(keep_land_s + CHANGE_LANE_DIST)) < 0) {
      acc = max_acc  / 2;
    }
    target_speed = compute_curr_speed();
  } else if (state == VehicleState::kLaneChangeLeft) {
    this->target_lane = this->target_lane - 1;
  } else if (state == VehicleState::kLaneChangeRight) {
    this->target_lane = this->target_lane + 1;
  } else if (state == VehicleState::kPrepareLaneChangeLeft) {
    target_speed = compute_curr_speed();
  } else if (state == VehicleState::kPrepareLaneChangeRight) {
    target_speed = compute_curr_speed();
  }
}

void MyVehicle::compute_path(const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                             vector<double> &path_x, vector<double> &path_y) {
  
  vector<double> x_for_spline;
  vector<double> y_for_spline;
  double ref_x, ref_y, ref_yaw;
  double ref_x_prev, ref_y_prev;
  double ref_speed, ref_acc;
  int prev_size = previous_path_x.size();

  ref_speed = 0;
  ref_acc = 0;
  if (prev_size < 2)
  {
    ref_x = x;
    ref_y = y;

    ref_x_prev = ref_x - cos(yaw);
    ref_y_prev = ref_y - sin(yaw);

    ref_speed = speed;

    keep_land_s = this->get_s();
    target_lane = this->get_lane();
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    ref_x_prev = previous_path_x[prev_size - 2];
    ref_y_prev = previous_path_y[prev_size - 2];

    ref_speed = sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / 0.02;

    if (previous_path_x.size() > 2) {
      double ref_x_prev_prev = previous_path_x[prev_size - 3];
      double ref_y_prev_prev = previous_path_y[prev_size - 3];

      
      double v2 = sqrt((ref_x_prev - ref_x_prev_prev) * (ref_x_prev - ref_x_prev_prev) + (ref_y_prev - ref_y_prev_prev) * (ref_y_prev - ref_y_prev_prev)) / 0.02;

      ref_acc = (ref_speed - v2) / 0.02;

      // std::cout << "Prev," << previous_path_x[prev_size - 3] << "," << previous_path_y[prev_size - 3] << std::endl;
      // std::cout << "Prev," << previous_path_x[prev_size - 2] << "," << previous_path_y[prev_size - 2] << std::endl;
      // std::cout << "Prev," << previous_path_x[prev_size - 1] << "," << previous_path_y[prev_size - 1] << std::endl;
    }
  }

  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
  x_for_spline.push_back(ref_x_prev);
  y_for_spline.push_back(ref_y_prev);
  x_for_spline.push_back(ref_x);
  y_for_spline.push_back(ref_y);

  // vector<double> tempSD;
  vector<double> tempXY;
  double start_change = target_speed * 3.0;

  // tempSD = getFrenet(ref_x, ref_y, ref_yaw, way_point_map);

  //cout << "ref," << ref_x << "," << ref_y << endl;
  //cout << "s + start_change," << s + start_change << endl;
  //cout << "tempSD," << tempSD[0] << "," << tempSD[1] << endl;
  // if (s + start_change > MAX_S) {
  //   if (tempSD[0] < MAX_S / 2) {
  //     tempSD[0] = tempSD[0] + MAX_S;
  //   }
  // }
  // tempSD[0] = (tempSD[0] + s + start_change) / 2;
  // tempSD[1] = (tempSD[1] + 2.0 + 4.0 * target_lane) / 2;
  // Vehicle zero_vehicle;
  // if (this->get_s_diff(zero_vehicle, -(keep_land_s + CHANGE_LANE_DIST)) > 0)
  // {
  //   tempSD[1] = 2.0 + 4.0 * target_lane;
  // }
  //cout << "tempSD," << tempSD[0] << "," << tempSD[1] << endl;
  // tempXY = getXY(tempSD[0], tempSD[1], way_point_map);
  //cout << "tempXY," << tempXY[0] << "," << tempXY[1] << endl;
  // x_for_spline.push_back(tempXY[0]);
  // y_for_spline.push_back(tempXY[1]);
  if (start_change < 40) {
    start_change = 40;
  }
  if (target_lane == get_lane()) {
    start_change = target_speed * 1.5;
    if (start_change < 30) {
      start_change = 30;
    }
  }
  // cout << "start_change" << start_change << endl;

  for (double step = start_change; step < start_change + 30.0 * 2 + 1; step += 30.0) {
    //cout << "LoopSD," << s + step << "," << (2.0 + 4.0 * target_lane) << endl;
    tempXY = getXY(s + step, (2.0 + 4.0 * target_lane), way_point_map);
    //cout << "LoopTempXY," << tempXY[0] << "," << tempXY[1] << endl;
    x_for_spline.push_back(tempXY[0]);
    y_for_spline.push_back(tempXY[1]);
  }

  for (int i = 0; i < x_for_spline.size(); i++) {
    double t_x = x_for_spline[i] - ref_x;
    double t_y = y_for_spline[i] - ref_y;

    x_for_spline[i] = (t_x * cos(-ref_yaw) - t_y * sin(-ref_yaw));
    y_for_spline[i] = (t_x * sin(-ref_yaw) + t_y * cos(-ref_yaw));
  }

  tk::spline s;
  s.set_points(x_for_spline, y_for_spline);

  for (int i = 0; i < previous_path_x.size(); i++)
  {
    path_x.push_back(previous_path_x[i]);
    path_y.push_back(previous_path_y[i]);
  }

  double x_add_on = 0;
  double y_add_on = 0;

  double vel = ref_speed;
  // std::cout << "init vel," << vel << ",init acc," << acc << std::endl;
  for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
    if (vel < target_speed) {
      vel += acc * 0.02;
      if (vel > target_speed) {
        vel = target_speed;
      }
    } else if (vel > target_speed) {
      vel -= acc * 0.02;
      if (vel < target_speed) {
        vel = target_speed;
      }
    } else {
    }

    double expected_diff = 0.02 * vel;
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
    } while (ratio < 0.999 || ratio > 1.001);

    double x_point = target_x;
    double y_point = s(x_point);
    x_add_on = x_point;
    y_add_on = y_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
    // std::cout << i << ",target," << target_speed << ",vel," << vel << ",acc," << acc << ",x," << x_point << ",y," << y_point << std::endl;

    path_x.push_back(x_point);
    path_y.push_back(y_point);
    // std::cout << "Running," << i << ",x," << x_point << ",y," << y_point << std::endl;
  }
  // for (int i = 0; i < path_x.size(); i++) {
  //   std::cout << "RunningAll," << i << ",x," << path_x[i] << ",y," << path_y[i] << std::endl;
  // }

}

bool MyVehicle::get_vehicle_ahead(Vehicle &vehicle)
{
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

bool MyVehicle::get_vehicle_behind(Vehicle &vehicle)
{
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

bool MyVehicle::get_left_vehicles(vector<Vehicle> &vehicles)
{
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

void MyVehicle::get_vehicles_ahead(vector<Vehicle> &vehicles) {
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
        }
      }
    }
  }
}

int MyVehicle::get_optimal_lane() {
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
  cout << "Optimal vehicle" << endl;
  for (int i = 0; i < vehicles.size(); i++) {
    cout << "ID," << vehicles[i].get_id();
    cout << ",X," << vehicles[i].get_x();
    cout << ",Y," << vehicles[i].get_y();
    cout << ",S," << vehicles[i].get_s();
    cout << ",D," << vehicles[i].get_d();
    cout << ",Speed," << vehicles[i].get_speed();
    cout << ",Yaw," << vehicles[i].get_yaw();
    cout << ",Lane," << vehicles[i].get_lane();
    cout << endl;
  }
  cout << "target lane: " << target_lane << endl;
  cout << "Optimal lane: " << optimal_lane << endl;
  return optimal_lane;
}

void MyVehicle::dump(vector<Vehicle> &left_vehicles, vector<Vehicle> &right_vehicles)
{
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
  cout << ",Speed," << get_speed();
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
    cout << ",Speed," << it->second.get_speed();
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
    cout << ",Speed," << left_vehicles[i].get_speed();
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
    cout << ",Speed," << right_vehicles[i].get_speed();
    cout << ",Yaw," << right_vehicles[i].get_yaw();
    cout << ",Lane," << right_vehicles[i].get_lane();
    cout << endl;
  }
  cout << "------------------------" << endl;
}