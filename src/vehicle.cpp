#include "vehicle.h"
#include <iostream>

using std::cout;
using std::endl;

Vehicle::Vehicle() {
  /**
   * Constructor.
   */
  this->id = -1;
  this->lane_width = 4.0;
  this->x = 0;
  this->y = 0;
  this->s = 0;
  this->d = 0;
  this->yaw = 0;
  this->v = 0;
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double v) : Vehicle() {
  /**
   * Constructor with parameters.
   * 
   * @param id - Unique identifier for the vehicle.
   * @param x - Global map x coordinate of the vehicle.
   * @param y - Global map y coordinate of the vehicle.
   * @param s - Frenet s coordinate of the vehicle.
   * @param d - Frenet d coordinate of the vehicle.
   * @param yaw - Orientation of the vehicle.
   * @param v - Velocity of the vehicle.
   */
  this->id = id;
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->v = v;
}

Vehicle::~Vehicle() {
  /**
   * Destructor.
   */
}

int Vehicle::get_lane() {
  /**
   * Calculate the lane ID.
   * 
   * @output the lane ID.
   */
  return (int) (this->d / this->lane_width);
}

int Vehicle::get_id() {
  /**
   * Output the unique identifier of the vehicle.
   * 
   * @output the unique identifier of the vehicle.
   */
  return id;
}

double Vehicle::get_x() {
  /**
   * Output the global map x coordinates of the vehicle.
   *
   * @output the global map x coordinates of the vehicle.
   */
  return x;
}

double Vehicle::get_y() {
  /**
   * Output the global map y coordinates of the vehicle.
   *
   * @output the global map y coordinates of the vehicle.
   */
  return y;
}

double Vehicle::get_s() {
  /**
   * Output the global map y coordinates of the vehicle.
   *
   * @output the global map y coordinates of the vehicle.
   */
  return s;
}

double Vehicle::get_d() {
  /**
   * Output the Frenet s coordinate.
   *
   * @output the Frenet s coordinate.
   */
  return d;
}

double Vehicle::get_yaw() {
  /**
   * Output the orientation of the vehicle.
   *
   * @output the orientation of the vehicle.
   */
  return yaw;
}

double Vehicle::get_velocity() {
  /**
   * Output the velocity of the vehicle.
   *
   * @output the velocity of the vehicle.
   */
  return v;
}

double Vehicle::get_s_diff(Vehicle &vehicle, double extra_offset) {
  /**
   * Calculate the distance of the input vehicle from the this vehicle
   * in Frenet s coordinate.
   *
   * @param vehicle - The vehicle for computing the distance.
   * @param extra_offset - The extra offset for this vehicle in Frenet s coordinate.
   *
   * @output the distance.
   */
  double offset = this->get_s() - vehicle.get_s() + extra_offset;
  if (offset > MAX_S / 2) {
    offset = offset - MAX_S;
  }
  if (offset < -MAX_S / 2) {
    offset = offset + MAX_S;
  }
  return offset;
}