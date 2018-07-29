#ifndef PLANNER_H_
#define PLANNER_H_

#include <map>
#include <vector>
#include "utils.h"
#include "spline.h"

using namespace std;

#define DT 0.02       // 50Hz
#define LEFT_LANE 0
#define SAME_LANE 1
#define RIGHT_LANE 2
#define LANE_CHANGE_LEFT -1
#define LANE_CHANGE_RIGHT 1

struct Map {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

/**
 * A behavior planner that uses prediction data to set the state of the ego vehicle to one of 5 values 
 * and generate a corresponding vehicle trajectory:
 * "KL" - Keep Lane
 * "LCL" / "LCR"- Lane Change Left / Lane Change Right
 */
class Planner {
public:
  Map map;

  enum State { KL, LCL, LCR };

  State state;

  // vehicle status
  int lane;
  double x;
  double y;
  double yaw;
  double speed;
  double s;
  double d;
  vector<double> prev_path_x;
  vector<double> prev_path_y;
  int prev_path_size;

  // reference values
  double ref_speed;         // m/s
  double max_acceleration;  // m/s^2
  int lanes_available;

  // best target values
  int best_lane;
  double best_speed;

  /**
  * Constructor
  */
  Planner(Map map, double ref_speed, double max_acceleration, int lanes_available, State state);

  /**
  * Destructor
  */
  virtual ~Planner();

  vector<vector<double>> next_path(const vector<vector<double>> & sensor_fusion);

  vector<State> successor_states();

  tk::spline generate_trajectory(int target_lane);

  void update_vehicle_state(double car_x, double car_y, double car_yaw, double car_speed, double car_d, double car_s,
                            const vector<double> & prev_path_x, const vector<double> & prev_path_y);
};

#endif  // PLANNER_H_