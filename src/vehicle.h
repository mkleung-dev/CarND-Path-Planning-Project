#ifndef VEHICLE_H
#define VEHICLE_H

#define MAX_S 6945.554

class Vehicle
{
protected:
  double lane_width;

  int id;
  double x, y;
  double s, d;
  double yaw;
  double speed;

public:
  Vehicle();
  Vehicle(int id, double x, double y, double s, double d, double yaw, double speed);
  virtual ~Vehicle();

  int get_lane();
  int get_id();
  double get_x();
  double get_y();
  double get_s();
  double get_d();
  double get_speed();
  double get_yaw();

  double get_s_diff(Vehicle &Vehicle, double extra_offset);


};

#endif // VEHICLE_H
