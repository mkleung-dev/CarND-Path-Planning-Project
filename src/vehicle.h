#ifndef VEHICLE_H
#define VEHICLE_H

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
  double get_x();
  double get_y();
  double get_s();
  double get_d();
  double get_speed();

};

#endif // VEHICLE_H
