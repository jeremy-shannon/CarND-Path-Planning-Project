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

double nearest_approach(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction) {
  double closest = 999999;
  for (int i = 0; i < N_SAMPLES; i++) {
    double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}

double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = 999999;
  for (auto prediction : predictions) {
    double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}

double nearest_approach_to_any_vehicle_in_lane(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = 999999;
  for (auto prediction : predictions) {
    double my_final_d = d_traj[d_traj.size() - 1];
    int my_lane = my_final_d / 4;
    vector<vector<double>> pred_traj = prediction.second;
    double pred_final_d = pred_traj[pred_traj.size() - 1][1];
    int pred_lane = pred_final_d / 4;
    if (my_lane == pred_lane) {
      double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
      if (current_dist < closest && current_dist < 120) {
        closest = current_dist;
      }
    }
  }
  return closest;
}

vector<double> velocities_for_trajectory(vector<double> traj) {
  // given a trajectory (a vector of positions), return the average velocity between each pair as a vector
  // also can be used to find accelerations from velocities, jerks from accelerations, etc.
  // (i.e. discrete derivatives)
  vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / DT);
  }
  return velocities;
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
  s_dot1 = (s1 - s2) / DT;
  s_dot2 = (s2 - s3) / DT;
  s_ddot = (s_dot1 - s_dot2) / DT;
  cost += fabs(s1 - target_s[0]) / SIGMA_S;
  cost += fabs(s_dot1 - target_s[1]) / SIGMA_S_DOT;
  cost += fabs(s_ddot - target_s[2]) / SIGMA_S_DDOT;
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

double in_lane_buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle_in_lane(s_traj, d_traj, predictions);
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
  double final_s_dot, total = 0;

  // cout << "DEBUG - s_dot: ";
  // for (double s_dot: s_dot_traj) {
  //   cout << s_dot << ", ";
  //   total += s_dot;
  // }
  // cout << "/DEBUG" << endl;
  // double avg_vel = total / s_dot_traj.size();

  final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
  // cout << "DEBUG - final s_dot: " << final_s_dot << endl;
  return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
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
  return logistic(avg_jerk / EXPECTED_JERK_IN_ONE_SEC );
}

double not_middle_lane_cost(vector<double> d_traj) {
  // penalize not shooting for middle lane (d = 6)
  double end_d = d_traj[d_traj.size()-1];
  return logistic(pow(end_d-6, 2));
}

double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {

  double total_cost = 0;
  double col = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
  double buf = buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
  double ilb = in_lane_buffer_cost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
  double eff = efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
  double nml = not_middle_lane_cost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;
  //double esl = exceeds_speed_limit_cost(s_traj) * SPEED_LIMIT_COST_WEIGHT;
  //double mas = max_accel_cost(s_traj) * MAX_ACCEL_COST_WEIGHT;
  //double aas = avg_accel_cost(s_traj) * AVG_ACCEL_COST_WEIGHT;
  //double mad = max_accel_cost(d_traj) * MAX_ACCEL_COST_WEIGHT;
  //double aad = avg_accel_cost(d_traj) * AVG_ACCEL_COST_WEIGHT;
  //double mjs = max_jerk_cost(s_traj) * MAX_JERK_COST_WEIGHT;
  //double ajs = avg_jerk_cost(s_traj) * AVG_JERK_COST_WEIGHT;
  //double mjd = max_jerk_cost(d_traj) * MAX_JERK_COST_WEIGHT;
  //double ajd = avg_jerk_cost(d_traj) * AVG_JERK_COST_WEIGHT;
  //double tdiff = time_diff_cost(target_time, actual_time) * TIME_DIFF_COST_WEIGHT;
  //double strajd = traj_diff_cost(s_traj, target_s) * TRAJ_DIFF_COST_WEIGHT;
  //double dtrajd = traj_diff_cost(d_traj, target_d) * TRAJ_DIFF_COST_WEIGHT;

  total_cost += col + buf + ilb + eff + nml;// + esl + mas + aas + mad + aad + mjs + ajs + mjd + ajd;

  // // DEBUG
  // cout << "costs - col: " << col << ", buf: " << buf << ", ilb: " << ilb << ", eff: " << eff << ", nml: " << nml; 
  // //cout << ", " << esl 
  // //cout << ", " << mas << ", " << aas << ", " << mad << ", " << aad;
  // //cout << ", " << mjs << ", " << ajs << ", " << mjd << ", " << ajd;
  // cout << "  ** ";
  // //cout << endl;
  // //cout << "total cost: " << total_cost << endl;

  return total_cost;
}

#endif