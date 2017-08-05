#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"

using namespace std;

// UTILITY FUNCTIONS

double logistic(double x){
  // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in 
  // the range[-infinity, infinity]. Useful for cost functions.
  return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // TODO
}



// COST FUNCTIONS

double time_diff_cost(double target_time, double actual_time) {
  // Penalizes trajectories that span a duration which is longer or shorter than the duration requested.
  return logistic(fabs(actual_time - target_time) / target_time);
}

double traj_diff_cost(vector<double> s_traj, vector<double> target_s) {
  // Penalizes trajectories whose s coordinate (and derivatives) differ from the goal. Target is s, s_dot, and s_ddot.
  // can be used for d trajectories as well (or any other 1-d trajectory)
  int s_end = s_traj.size();
  double s1, s2, s3, s_dot1, s_dot2, s_ddot, cost = 0;
  s1 = s_traj[s_end - 1];
  s2 = s_traj[s_end - 2];
  s3 = s_traj[s_end - 3];
  s_dot1 = (s1 - s2) / timestep;
  s_dot2 = (s2 - s3) / timestep;
  s_ddot = (s_dot1 - s_dot2) / timestep;
  cost += fabs(s1 - target_s[0]) / sigma_s[0];
  cost += fabs(s_dot1 - target_s[1]) / sigma_s[1];
  cost += fabs(s_ddot - target_s[2]) / sigma_s[2];
  return logistic(cost);
}

double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Binary cost function which penalizes collisions.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  if (nearest < 2 * VEHICLE_RADIUS) {
    return 1;
  } else { 
    return 0;
  }
}

double buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double exceeds_speed_limit_cost(vector<double> s_traj) {
  // Penalty if ds/dt for any two points in trajectory is greater than SPEED_LIMIT
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  for (double s_dot : s_dot_traj) {
    if (s_dot > SPEED_LIMIT) {
      return 1;
    }
  }
  return 0;
}

double efficiency_cost(vector<double> s_traj) {
  // Rewards high average speeds.
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  double total = 0;
  for (double s_dot: s_dot_traj) {
    total += s_dot;
  }
  double avg_vel = total / s_dot_traj.size();
  return logistic(2 * (SPEED_LIMIT - avg_vel) / avg_vel);
} 

double max_accel_cost(vector<double> s_traj) {
  // Penalize exceeding MAX_INSTANTANEOUS_ACCEL
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  for (double s_ddot : s_ddot_traj) {
    if (s_ddot > MAX_INSTANTANEOUS_ACCEL) {
      return 1;
    }
  }
  return 0;
}

double avg_accel_cost(vector<double> s_traj) {
  // Penalize higher average acceleration
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  double total = 0;
  for (double s_ddot: s_ddot_traj) {
    total += s_ddot;
  }
  double avg_accel = total / s_ddot_traj.size();
  return logistic(avg_accel / EXPECTED_ACC_IN_ONE_SEC );
}

double max_jerk_cost(vector<double> s_traj) {
  // Penalize exceeding MAX_INSTANTANEOUS_JERK
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
  for (double s_dddot : s_dddot_traj) {
    if (s_dddot > MAX_INSTANTANEOUS_JERK) {
      return 1;
    }
  }
  return 0;
}

double avg_jerk_cost(vector<double> s_traj) {
  // Penalize higher average jerk
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
  double total = 0;
  for (double s_dddot: s_dddot_traj) {
    total += s_dddot;
  }
  double avg_jerk = total / s_dddot_traj.size();
  return logistic(avg_a / EXPECTED_ACC_IN_ONE_SEC );
}

double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions, vector<double> target_s, vector<double> target_d, double target_time, double actual_time) {

  double total_cost = 0;

  total_cost += time_diff_cost(target_time, actual_time) * TIME_DIFF_COST_WEIGHT;
  total_cost += traj_diff_cost(s_traj, target_s) * TRAJ_DIFF_COST_WEIGHT;
  total_cost += traj_diff_cost(d_traj, target_d) * TRAJ_DIFF_COST_WEIGHT;
  total_cost += collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
  total_cost += buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
  total_cost += exceeds_speed_limit_cost(s_traj) * SPEED_LIMIT_COST_WEIGHT;
  total_cost += efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
  total_cost += max_accel_cost(s_traj) * MAX_ACCEL_COST_WEIGHT;
  total_cost += avg_accel_cost(s_traj) * AVG_ACCEL_COST_WEIGHT;
  total_cost += max_accel_cost(d_traj) * MAX_ACCEL_COST_WEIGHT;
  total_cost += avg_accel_cost(d_traj) * AVG_ACCEL_COST_WEIGHT;
  total_cost += max_jerk_cost(s_traj) * MAX_JERK_COST_WEIGHT;
  total_cost += avg_jerk_cost(s_traj) * AVG_JERK_COST_WEIGHT;
  total_cost += max_jerk_cost(d_traj) * MAX_JERK_COST_WEIGHT;
  total_cost += avg_jerk_cost(d_traj) * AVG_JERK_COST_WEIGHT;

  return total_cost;
}

#endif