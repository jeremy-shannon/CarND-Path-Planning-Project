#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"

using namespace std;

double logistic(double x){
  // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in 
  // the range[-infinity, infinity]. Useful for cost functions.
  return 2.0 / (1 + exp(-x)) - 1.0;
}

// COST FUNCTIONS

double time_diff_cost(double target_time, double actual_time) {
  // Penalizes trajectories that span a duration which is longer or shorter than the duration requested.
  return logistic(fabs(actual_time - target_time) / target_time);
}

double traj_diff_cost(vector<double> s_traj, vector<double> target_s, vector<double> sigma_s, double timestep) {
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
    return 1.0;
  } else { 
    return 0.0;
  }
}

double buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double exceeds_speed_limit_cost(traj, target_vehicle, delta, T, predictions) :
pass

def efficiency_cost(traj, target_vehicle, delta, T, predictions) :
"""
Rewards high average speeds.
"""
s, _, t = traj
s = to_equation(s)
avg_v = float(s(t)) / t
targ_s, _, _, _, _, _ = predictions[target_vehicle].state_in(t)
targ_v = float(targ_s) / t
return logistic(2 * float(targ_v - avg_v) / avg_v)

def max_accel_cost(traj, target_vehicle, delta, T, predictions) :
s, d, t = traj
s_dot = differentiate(s)
s_d_dot = differentiate(s_dot)
a = to_equation(s_d_dot)
total_acc = 0
dt = float(T) / 100.0
for i in range(100) :
t = dt * i
acc = a(t)
total_acc += abs(acc*dt)
acc_per_second = total_acc / T

return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC)

def total_accel_cost(traj, target_vehicle, delta, T, predictions) :
s, d, t = traj
s_dot = differentiate(s)
s_d_dot = differentiate(s_dot)
a = to_equation(s_d_dot)
all_accs = [a(float(T) / 100 * i) for i in range(100)]
max_acc = max(all_accs, key = abs)
if abs(max_acc) > MAX_ACCEL: return 1
else: return 0


def max_jerk_cost(traj, target_vehicle, delta, T, predictions) :
s, d, t = traj
s_dot = differentiate(s)
s_d_dot = differentiate(s_dot)
jerk = differentiate(s_d_dot)
jerk = to_equation(jerk)
all_jerks = [jerk(float(T) / 100 * i) for i in range(100)]
max_jerk = max(all_jerks, key = abs)
if abs(max_jerk) > MAX_INSTANTANEOUS_JERK: return 1
else: return 0

def total_jerk_cost(traj, target_vehicle, delta, T, predictions) :
s, d, t = traj
s_dot = differentiate(s)
s_d_dot = differentiate(s_dot)
jerk = to_equation(differentiate(s_d_dot))
total_jerk = 0
dt = float(T) / 100.0
for i in range(100) :
t = dt * i
j = jerk(t)
total_jerk += abs(j*dt)
jerk_per_second = total_jerk / T
return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC)

*/





#endif