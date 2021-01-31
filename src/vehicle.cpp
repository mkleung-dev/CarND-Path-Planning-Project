#include "vehicle.h"

#include <iostream>
#include "helpers.h"
#include "spline.h"

Vehicle::Vehicle() {
  lane_available = 3;
  lane_width = 4.0;
  max_velocity = 50.0 / 2.24;
  max_acc = 9.5;
  max_jerk = 9.5;
  target_speed = max_velocity;
}

Vehicle::Vehicle(WayPointMap &way_point_map) : Vehicle() {
  this->way_point_map = way_point_map;
}

Vehicle::~Vehicle() {

}

vector<VehicleState> Vehicle::successor_states() {
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

void Vehicle::update_position(double x, double y,
                              double s, double d,
                              double yaw, double speed)
{
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  std::cout << "update,x," << x << ",y," << y << ",s," << s << ",d," << d << ",yaw," << yaw << ",speed," << speed << std::endl;
}

void Vehicle::compute_path(const vector<double> &previous_path_x,
                           const vector<double> &previous_path_y,
                           vector<double> &path_x,
                           vector<double> &path_y) {
  
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
    }  
  }

  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
  x_for_spline.push_back(ref_x_prev);
  y_for_spline.push_back(ref_y_prev);
  x_for_spline.push_back(ref_x);
  y_for_spline.push_back(ref_y);

  int lane = 1;
  vector<double> tempXY;
  for (double step = 30.0; step < 91.0; step += 30.0) {
    tempXY = getXY(s + step, (2.0 + 4.0 * lane), way_point_map);
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

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_x);

  double x_add_on = 0;

  double vel = ref_speed;
  double acc = ref_acc;
  std::cout << "init vel," << vel << ",init acc," << acc << std::endl;
  for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

    if (target_speed > vel) {
      acc += max_jerk * 0.02;
      if (acc > max_acc) {
        acc = max_acc;
      }
      vel += acc * 0.02;
      if (vel > target_speed) {
        vel = target_speed;
      }
    } else if (target_speed < vel) {
      acc -= max_jerk * 0.02;
      if (acc < -max_acc) {
        acc = -max_acc;
      }
      vel -= acc * 0.02;
      if (vel < target_speed) {
        vel = target_speed;
      }
    }
    std::cout << "vel," << vel << ",acc," << acc << std::endl;

    double N = (target_dist / (0.02 * vel));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    path_x.push_back(x_point);
    path_y.push_back(y_point);
  }
  for (int i = 0; i < path_x.size(); i++) {
    std::cout << i << ",x," << path_x[i] << ",y," << path_y[i] << std::endl;
  }

}