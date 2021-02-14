#ifndef VEHICLE_H
#define VEHICLE_H

#define MAX_S 6945.554

/**
 * Class to store vehicle data.
 */ 
class Vehicle
{
protected:
  // The lane width
  double lane_width;

  // Unique identifier of the vehicle.
  int id;
  // Vehicle position in XY coordinates;
  double x, y;
  // Vehicle position in Frenet coordinates;
  double s, d;
  // Vehicle position in Frenet coordinates;
  double yaw;
  double v;

public:
  /**
   * Constructor.
   */ 
  Vehicle();
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
  Vehicle(int id, double x, double y, double s, double d, double yaw, double v);
  /**
   * Destructor.
   */
  virtual ~Vehicle();

  /**
   * Calculate the lane ID.
   * 
   * @output the lane ID.
   */
  int get_lane();

  /**
   * Output the unique identifier of the vehicle.
   * 
   * @output the unique identifier of the vehicle.
   */
  int get_id();

  /**
   * Output the global map x coordinates of the vehicle.
   *
   * @output the global map x coordinates of the vehicle.
   */
  double get_x();

  /**
   * Output the global map y coordinates of the vehicle.
   *
   * @output the global map y coordinates of the vehicle.
   */
  double get_y();

  /**
   * Output the global map y coordinates of the vehicle.
   *
   * @output the global map y coordinates of the vehicle.
   */
  double get_s();

  /**
   * Output the Frenet s coordinate.
   *
   * @output the Frenet s coordinate.
   */
  double get_d();

  /**
   * Output the orientation of the vehicle.
   *
   * @output the orientation of the vehicle.
   */
  double get_yaw();

  /**
   * Output the velocity of the vehicle.
   *
   * @output the velocity of the vehicle.
   */
  double get_velocity();

  /**
   * Calculate the distance of the input vehicle from the this vehicle
   * in Frenet s coordinate.
   *
   * @param vehicle - The vehicle for computing the distance.
   * @param extra_offset - The extra offset for this vehicle in Frenet s coordinate.
   *
   * @output the distance.
   */
  double get_s_diff(Vehicle &Vehicle, double extra_offset);
};

#endif // VEHICLE_H
