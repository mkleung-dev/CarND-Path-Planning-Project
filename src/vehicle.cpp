#include "vehicle.h"
#include <iostream>

using std::cout;
using std::endl;

Vehicle::Vehicle() {
  this->id = -1;
  this->lane_width = 4.0;
  this->x = 0;
  this->y = 0;
  this->s = 0;
  this->d = 0;
  this->yaw = 0;
  this->speed = 0;
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double speed) : Vehicle() {
  //cout << "Vehicle,id," << id << ",x," << x << ",y," << y << ",s," << s << ",d," << d << ",yaw," << yaw << ",speed," << speed << endl;
  this->id = id;
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
}

Vehicle::~Vehicle() {
}

int Vehicle::get_lane() {
  return (int) (this->d / this->lane_width);
}

int Vehicle::get_id() {
  return id;
}

double Vehicle::get_x() {
  return x;
}

double Vehicle::get_y() {
  return y;
}

double Vehicle::get_s() {
  return s;
}

double Vehicle::get_d() {
  return d;
}

double Vehicle::get_yaw() {
  return yaw;
}

double Vehicle::get_speed() {
  return speed;
}

double Vehicle::get_s_diff(Vehicle &vehicle, double extra_offset) {
  double offset = this->get_s() - vehicle.get_s() + extra_offset;
  if (offset > MAX_S / 2) {
    offset = offset - MAX_S;
  }
  if (offset < -MAX_S / 2) {
    offset = offset + MAX_S;
  }
  return offset;
}