#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "planner.h"

/**
 * Initializes Planner
 */

Planner::Planner(Map map, double ref_speed, double max_acceleration, int lanes_available, State state=KL) {
  this->map = map;
  this->ref_speed = ref_speed;
  this->max_acceleration = max_acceleration;
  this->lanes_available = lanes_available;
  this->state = state;
}

Planner::~Planner() {}

vector<vector<double>> Planner::next_path(const vector<vector<double>> & sensor_fusion) {
  // INPUT: A 2d vector of cars and then that car's 
  // [    car's unique ID, 
  //      car's x position in map coordinates, 
  //      car's y position in map coordinates, 
  //      car's x velocity in m/s, 
  //      car's y velocity in m/s, 
  //      car's s position in frenet coordinates, 
  //      car's d position in frenet coordinates].
  // OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.
  
  vector<double> closest_dist_ahead(3, numeric_limits<double>::max());
  vector<double> closest_dist_behind(3, numeric_limits<double>::max());
  vector<int> closest_ahead(3, -1);
  vector<int> closest_behind(3, -1);
  double safe_dist = this->speed * 2;   // 2 second rule
  double dt = prev_path_size * DT;
  
  for (size_t i = 0; i < sensor_fusion.size(); i++ ) {
    float obs_car_d = sensor_fusion[i][6];
    int obs_car_lane = utils::getLane(obs_car_d);
    int lane_index = obs_car_lane - this->lane + 1;    // 0: left lane, 1: same lane, 2: right lane
    if (lane_index < LEFT_LANE || lane_index > RIGHT_LANE) continue;

    // Find car speed.
    double obs_car_vx = sensor_fusion[i][3];
    double obs_car_vy = sensor_fusion[i][4];
    double obs_car_speed = sqrt(obs_car_vx*obs_car_vx + obs_car_vy*obs_car_vy);
    double obs_car_s = sensor_fusion[i][5];
    
    // Predict observation car s
    obs_car_s += obs_car_speed * dt;
    double distance = obs_car_s - this->s;

    if (distance >= 0 && distance < closest_dist_ahead[lane_index] && distance < safe_dist) {
      closest_dist_ahead[lane_index] = distance;
      closest_ahead[lane_index] = i;
    } else if (distance < 0 && abs(distance) < closest_dist_behind[lane_index] 
                && abs(distance) < obs_car_speed * 2) {
      closest_dist_behind[lane_index] = distance;
      closest_behind[lane_index] = i;
    }
  }

  for (size_t i = 0; i < 3; i++) {
    cout << "ahead: " << closest_ahead[i] << " ";
  }
  cout << endl;
  for (size_t i = 0; i < 3; i++) {
    cout << "behind: " << closest_behind[i] << " ";
  }
  cout << endl;
    
  vector<State> possible_successor_states = successor_states();

  // find the trajectory of each successor state with minimum cost
  float min_cost = numeric_limits<double>::max();
  tk::spline best_trajectory;
  for(size_t i = 0; i<possible_successor_states.size(); ++i) {
    State successor_state = possible_successor_states[i];

    double target_speed;
    int target_lane;
    double cost_for_state;

    switch (successor_state) {
    case KL:
      if (closest_ahead[SAME_LANE] == -1) {
        target_speed = min(this->speed + this->max_acceleration, this->ref_speed);
      } else {
        target_speed = this->speed - this->max_acceleration * 0.8;
      }

      target_lane = this->lane;
      cost_for_state = 0;
      break;
    case LCL:
      if (closest_ahead[LEFT_LANE] == -1 && closest_behind[LEFT_LANE] == -1) {
        target_speed = min(this->speed + this->max_acceleration, this->ref_speed); 
      } else {
        target_speed = -this->ref_speed;
      }

      target_lane = this->lane + LANE_CHANGE_LEFT;
      cost_for_state = 0.1;
      break;
    case LCR:
      if (closest_ahead[RIGHT_LANE] == -1 && closest_behind[RIGHT_LANE] == -1) {
        target_speed = min(this->speed + this->max_acceleration, this->ref_speed); 
      } else {
        target_speed = -this->ref_speed;
      }

      target_lane = this->lane + LANE_CHANGE_RIGHT;
      cost_for_state = 0.2;
      break;
    }

    cout << "state - " << successor_state << "| target lane - " << target_lane << "| target speed - " << target_speed << endl; 

    // calculate the "cost" associated with next state.
    cost_for_state -= target_speed;
    
    if (cost_for_state < min_cost) {
      this->best_lane = target_lane;
      this->best_speed = target_speed;
      min_cost = cost_for_state;
    }
  }

  cout << "best lane - " << this->best_lane << "| best speed - " << this->best_speed << endl; 

  tk::spline next_trajectory = generate_trajectory(best_lane);

  vector<double> next_x_vals, next_y_vals;

  for (size_t i = 0; i < this->prev_path_size; i++) {
    next_x_vals.push_back(this->prev_path_x[i]);
    next_y_vals.push_back(this->prev_path_y[i]);
  }

  // generate waypoints in 2s of future
  double next_x = this->best_speed * 2;
  double next_y = next_trajectory(next_x);
  double next_dist = utils::distance(next_x, next_y, 0, 0);
  double prev_x = 0;

  for (size_t i = 1; i <= 50 - this->prev_path_size; i++) {
    double N = (next_dist/(DT*this->best_speed));
    double x_point = prev_x + next_x / N;
    double y_point = next_trajectory(x_point);

    prev_x = x_point;

    double new_x_point = x_point * cos(this->yaw) - y_point * sin(this->yaw) + this->x;
    double new_y_point = x_point * sin(this->yaw) + y_point * cos(this->yaw) + this->y;

    next_x_vals.push_back(new_x_point);
    next_y_vals.push_back(new_y_point);
  }

  return {next_x_vals, next_y_vals};
}

vector<Planner::State> Planner::successor_states() {
    // Provides the possible next states given the current state for the FSM 
    // discussed in the course, with the exception that lane changes happen 
    // instantaneously, so LCL and LCR can only transition back to KL.

    vector<State> states;
    states.push_back(KL);
    switch (this->state) {
      case KL:
        if (this->lane > 0) states.push_back(LCL);
        if (this->lane < this->lanes_available - 1) states.push_back(LCR);
        break;
      default:
        break;
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

tk::spline Planner::generate_trajectory(int target_lane) {
  vector<double> trajectory_x;
  vector<double> trajectory_y;

  if (this->prev_path_size < 2) {
    double prev_car_x = this->x - cos(this->yaw);
    double prev_car_y = this->y - sin(this->yaw);
    trajectory_x.push_back(prev_car_x);
    trajectory_x.push_back(this->x);
    trajectory_y.push_back(prev_car_y);
    trajectory_y.push_back(this->y);
  } else {
    trajectory_x.push_back(this->prev_path_x[this->prev_path_size-2]);
    trajectory_x.push_back(this->prev_path_x[this->prev_path_size-1]);
    trajectory_y.push_back(this->prev_path_y[this->prev_path_size-2]);
    trajectory_y.push_back(this->prev_path_y[this->prev_path_size-1]);
  }

  vector<double> next_wp0 = utils::getXY(this->s+30, (2+4*target_lane),
                                  map.map_waypoints_s, map.map_waypoints_x, map.map_waypoints_y);
  vector<double> next_wp1 = utils::getXY(this->s+60, (2+4*target_lane),
                                  map.map_waypoints_s, map.map_waypoints_x, map.map_waypoints_y);
  vector<double> next_wp2 = utils::getXY(this->s+90, (2+4*target_lane),
                                  map.map_waypoints_s, map.map_waypoints_x, map.map_waypoints_y);

  trajectory_x.push_back(next_wp0[0]);
  trajectory_x.push_back(next_wp1[0]);
  trajectory_x.push_back(next_wp2[0]);

  trajectory_y.push_back(next_wp0[1]);
  trajectory_y.push_back(next_wp1[1]);
  trajectory_y.push_back(next_wp2[1]);

  utils::global2CarCoordinate(this->x, this->y, this->yaw, trajectory_x, trajectory_y);  

  tk::spline s;
  s.set_points(trajectory_x, trajectory_y);

  return s;
}

void Planner::update_vehicle_state(double car_x, double car_y, double car_yaw, double car_speed, double car_d, double car_s,
                                   const vector<double> & prev_path_x, const vector<double> & prev_path_y) {
    // Update current vehicle state 

  this->lane = utils::getLane(car_d);
  this->x = car_x;
  this->y = car_y;
  this->yaw = utils::deg2rad(car_yaw);
  this->speed = utils::mph2mps(car_speed);
  this->s = car_s;
  this->d = car_d;
  this->prev_path_x = prev_path_x;
  this->prev_path_y = prev_path_y;

  // if previous path have more than 2 points, planner will generate the trajectory based on previous path
  this->prev_path_size = prev_path_x.size();
  if (this->prev_path_size > 2) {
    double ref_x = this->prev_path_x[this->prev_path_size-1];
    double ref_y = this->prev_path_y[this->prev_path_size-1];
    double prev_ref_x = this->prev_path_x[this->prev_path_size-2];
    double prev_ref_y = this->prev_path_y[this->prev_path_size-2];
    double ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    this->x = ref_x;
    this->y = ref_y;
    this->yaw = ref_yaw;
  }

  cout<<"car speed: " << speed << "| car s: " << car_s << "| car d: " << 
  car_d << "| car lane: " << lane << endl;
}