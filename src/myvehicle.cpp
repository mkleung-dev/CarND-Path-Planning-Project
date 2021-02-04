#include "myvehicle.h"

#include <iostream>
#include "helpers.h"
#include "spline.h"

using std::vector;
using std::map;
using std::cout;
using std::endl;

MyVehicle::MyVehicle() : Vehicle() {
  lane_available = 3;
  max_velocity = 44 / 2.24;
  max_acc = 3;
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
  std::cout << "update,x," << x << ",y," << y << ",s," << s << ",d," << d << ",yaw," << yaw << ",speed," << speed << std::endl;
}

void MyVehicle::update_state() {
  Vehicle vehicle;

  if (state == VehicleState::kKeepLane) {
    vector<double> lane_speeds = get_lane_speed();
    int max_speed_lane = this->target_lane;
    double max_speed = lane_speeds[max_speed_lane];
    for (int i = 0; i < lane_speeds.size(); i++) {
      if (lane_speeds[i] > max_speed + 0.001) {
        max_speed = lane_speeds[i];
        max_speed_lane = i;
      }
    }
    cout << lane_speeds[0] << " " << lane_speeds[1] << " " << lane_speeds[2] << endl;
    cout << "self_lane: " << this->get_lane() << " max_lane: " << max_speed_lane << endl;
    if (max_speed_lane < this->target_lane && this->get_s() > keep_land_s + 30) {
      state = VehicleState::kPrepareLaneChangeLeft;
    }
    if (max_speed_lane > this->target_lane && this->get_s() > keep_land_s + 30) {
      state = VehicleState::kPrepareLaneChangeRight;
    }
  } else if (state == VehicleState::kPrepareLaneChangeLeft) {
    Vehicle left_vehicle;
    if (get_left_vehicle(left_vehicle)) {
      if (this->get_s() > left_vehicle.get_s() + 10 ||
          this->get_s() < left_vehicle.get_s() - 10)
      {
        state = VehicleState::kLaneChangeLeft;
      }
    }
  } else if (state == VehicleState::kPrepareLaneChangeRight) {
    Vehicle right_vehicle;
    if (get_right_vehicle(right_vehicle)) {
      if (this->get_s() > right_vehicle.get_s() + 10 ||
          this->get_s() < right_vehicle.get_s() - 10)
      {
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

  if (state == VehicleState::kKeepLane) {
    cout << "VehicleState::kKeepLane" << endl;
  } else if (state == VehicleState::kLaneChangeLeft) {
    cout << "VehicleState::kLaneChangeLeft" << endl;
  } else if (state == VehicleState::kLaneChangeRight) {
    cout << "VehicleState::kLaneChangeRight" << endl;
  } else if (state == VehicleState::kPrepareLaneChangeLeft) {
    cout << "VehicleState::kPrepareLaneChangeLeft" << endl;
  } else if (state == VehicleState::kPrepareLaneChangeRight) {
    cout << "VehicleState::kPrepareLaneChangeRight" << endl;
  }
}

void MyVehicle::update_action() {
  if (state == VehicleState::kKeepLane) {
    Vehicle vehicle_ahead;
    target_speed = max_velocity;
    if (get_vehicle_ahead(vehicle_ahead))
    {
      cout << "get_vehicle_ahead," << vehicle_ahead.get_speed() << endl;
      if (vehicle_ahead.get_s() - this->get_s() < this->get_speed() * 3)
      {
        target_speed = vehicle_ahead.get_speed();
      }
    }
  } else if (state == VehicleState::kLaneChangeLeft) {
    this->target_lane = this->target_lane - 1;
  } else if (state == VehicleState::kLaneChangeRight) {
    this->target_lane = this->target_lane + 1;
  } else if (state == VehicleState::kPrepareLaneChangeLeft) {
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    Vehicle left_vehicle;
    target_speed = max_velocity;
    if (get_vehicle_ahead(vehicle_ahead))
    {
      cout << "get_vehicle_ahead," << vehicle_ahead.get_speed() << endl;
      if (vehicle_ahead.get_s() - this->get_s() < this->get_speed() * 3)
      {
        target_speed = vehicle_ahead.get_speed();
      }
    }
  } else if (state == VehicleState::kPrepareLaneChangeRight) {
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    Vehicle right_vehicle;
    target_speed = max_velocity;
    if (get_vehicle_ahead(vehicle_ahead))
    {
      cout << "get_vehicle_ahead," << vehicle_ahead.get_speed() << endl;
      if (vehicle_ahead.get_s() - this->get_s() < this->get_speed() * 3)
      {
        target_speed = vehicle_ahead.get_speed();
      }
    }
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

  vector<double> tempXY;
  for (double step = 40.0; step < 121.0; step += 40.0) {
    tempXY = getXY(s + step, (2.0 + 4.0 * target_lane), way_point_map);
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
      vel += max_acc * 0.02;
      if (vel > target_speed) {
        vel = target_speed;
      }
    } else if (vel > target_speed) {
      vel -= max_acc * 0.02;
      if (vel < target_speed) {
        vel = target_speed;
      }
    } else {
    }


    double target_x = x_add_on + 0.02 * vel;
    double target_y = s(target_x);

    double diff_x = target_x - x_add_on;
    double diff_y = target_y - y_add_on;

    double base = sqrt(diff_x * diff_x + diff_y * diff_y);
    double ratio = diff_x / base;

    target_x = x_add_on + 0.02 * vel * ratio;
    target_y = s(target_x);

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
    std::cout << "Running," << i << ",x," << x_point << ",y," << y_point << std::endl;
  }
  for (int i = 0; i < path_x.size(); i++) {
    std::cout << "RunningAll," << i << ",x," << path_x[i] << ",y," << path_y[i] << std::endl;
  }

}

bool MyVehicle::get_vehicle_ahead(Vehicle &vehicle)
{
  bool vehicle_found = false;
  double min_s = -1;
  for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
    if (it->second.get_lane() == this->get_lane() &&
        it->second.get_s() > this->get_s()) {
      if (min_s < 0 || it->second.get_s() < min_s) {
        min_s = it->second.get_s();
        vehicle = it->second;
        vehicle_found = true;
      }
    }
  }
  return vehicle_found;
}

bool MyVehicle::get_vehicle_behind(Vehicle &vehicle)
{
  bool vehicle_found = false;
  double max_s = -1;
  for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
    if (it->second.get_lane() == this->get_lane() &&
        it->second.get_s() < this->get_s()) {
      if (max_s < 0 || it->second.get_s() > max_s) {
        max_s = it->second.get_s();
        vehicle = it->second;
        vehicle_found = true;
      }
    }
  }
  return vehicle_found;
}

bool MyVehicle::get_left_vehicle(Vehicle &vehicle)
{
  bool vehicle_found = false;
  double min_s = -1;
  if (this->get_lane() > 0) {
    for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
      if (it->second.get_lane() == (this->get_lane() - 1) &&
          it->second.get_s() > this->get_s() - 20) {
        if (min_s < 0 || it->second.get_s() < min_s) {
          min_s = it->second.get_s();
          vehicle = it->second;
          vehicle_found = true;
        }
      }
    }
  }
  return vehicle_found;
}
bool MyVehicle::get_right_vehicle(Vehicle &vehicle)
{
  bool vehicle_found = false;
  double min_s = -1;
  if (this->get_lane() < lane_available) {
    for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
      if (it->second.get_lane() == (this->get_lane() + 1) &&
          it->second.get_s() > this->get_s() - 20) {
        if (min_s < 0 || it->second.get_s() < min_s) {
          min_s = it->second.get_s();
          vehicle = it->second;
          vehicle_found = true;
        }
      }
    }
  }
  return vehicle_found;
}

vector<double> MyVehicle::get_lane_speed()
{
  vector<double> lane_speeds;
  lane_speeds.resize(lane_available);
  for (int i = 0; i < lane_speeds.size(); i++) {
    lane_speeds[i] = max_velocity;
  }
  for (map<int, Vehicle>::iterator it = other_vehicles.begin(); it != other_vehicles.end(); it++) {
    if (0 <= it->second.get_lane() && it->second.get_lane() < lane_speeds.size()) {
      if (it->second.get_s() > this->get_s()) {
        if (it->second.get_speed() < lane_speeds[it->second.get_lane()]) {
          lane_speeds[it->second.get_lane()] = it->second.get_speed();
        }
      }
    }
  }

  return lane_speeds;
}