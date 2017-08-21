#include "vehicle.h"
#include "constants.h"
#include "jmt.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <random>
#include <algorithm>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd) {

  this->s    = s;         // s position
  this->s_d  = s_d;       // s dot - velocity in s
  this->s_dd = s_dd;      // s dot-dot - acceleration in s
  this->d    = d;         // d position
  this->d_d  = d_d;       // d dot - velocity in d
  this->d_dd = d_dd;      // d dot-dot - acceleration in d
  state = "CS";

}

Vehicle::~Vehicle() {}

vector<vector<double>> Vehicle::get_best_frenet_trajectory(map<int, vector<vector<double>>> predictions, double duration) {
  
  // NOTE: THIS METHOD IS FROM AN ALTERNATE IMPLEMENTATION AND NO LONGER USED
  bool car_to_left = false, car_to_right = false, car_just_ahead = false;
  update_available_states(car_to_left, car_to_right);

  // // DEBUG
  // cout << "available states: "; 
  // for (auto st: available_states) cout << st << " ";
  // cout << endl; 

  vector<vector<double>> best_frenet_traj, best_target;
  double best_cost = 999999;
  string best_traj_state = "";

  /*
  for (string state : available_states) {

    // target state (s and d) and time
    vector<vector<double>> target_s_and_d = get_target_for_state(state, predictions);
    double target_time = duration;

    // DEBUG
    cout << "target s&d for state " << state << ": ";
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
        cout << target_s_and_d[i][j];
        if (j != 2) cout << ", ";
      }
      cout << "; ";
    }
    cout << endl;

    // Perturb trajectories, first by duration
    for (int i = -NUM_TIMESTEPS_TO_PERTURB; i <= NUM_TIMESTEPS_TO_PERTURB; i++) {
      double perturbed_time = target_time + (i * SIGMA_T);

      // Perturb by sigma s and d values
      for (int i = 0; i < NUM_RANDOM_TRAJ_TO_GEN; i++) {
        vector<vector<double>> perturbed_target = perturb(target_s_and_d);

         // DEBUG
        cout << "perturbed target s&d for time " << perturbed_time << ": ";
        for (int i = 0; i < 2; i++) {
          for (int j = 0; j < 3; j++) {
            cout << perturbed_target[i][j];
            if (j != 2) cout << ", ";
          }
          cout << "; ";
        }
        cout << endl;

        vector<vector<double>> possible_traj = generate_traj_for_target(perturbed_target, perturbed_time);

        // // DEBUG
        // cout << "possible_traj: ";
        // for (int i = 0; i < N_SAMPLES; i += N_SAMPLES/3-1) {
        //   cout << "(" << possible_traj[0][i] <<  "," << possible_traj[1][i] << ") ";
        // }
        // cout << endl;

        double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions, target_s_and_d[0], target_s_and_d[1], target_time, perturbed_time);

        // // DEBUG
        // cout << "cost: " << current_cost << endl;

        if (current_cost < best_cost) {
          best_cost = current_cost;
          best_frenet_traj = possible_traj;
          best_traj_state = state;
          best_target = perturbed_target;
        }
      }
    }
  } 
  
  // have to call generate_traj_for_target to reset this->s_coeffs etc.
  generate_traj_for_target(best_target, duration);
  */
  
  // DEBUG - ONLY KEEP LANE AND NO PERTURB
  state = "KL";
  best_target = get_target_for_state(state, predictions, duration, car_just_ahead);
  best_frenet_traj = generate_traj_for_target(best_target, duration);

  // // DEBUG
  // cout << "chosen state: " << best_traj_state << ", cost: " << best_cost << ", ";
  // cout << "target (s,sd,sdd - d,dd,ddd): (";
  // for (int i = 0; i < 2; i++) {
  //   for (int j = 0; j < 3; j++) {
  //     cout << best_target[i][j];
  //     if (j != 2) cout << ", ";
  //   }
  //   cout << "; ";
  // }
  // cout << ")" << endl;

  // // DEBUG
  // cout << "best frenet trajectory (s,d):" << endl;
  // for (int i = 0; i < best_frenet_traj[0].size(); i++) {
  //   cout << best_frenet_traj[0][i] << ", " << best_frenet_traj[1][i] << endl;
  // }
  // cout << endl << endl;

  return best_frenet_traj;
}

void Vehicle::update_available_states(bool car_to_left, bool car_to_right) {
  /*  Updates the available "states" based on the current state:
  "KL" - Keep Lane
   - The vehicle will attempt to drive its target speed, unless there is 
     traffic in front of it, in which case it will slow down.
  "LCL" or "LCR" - Lane Change Left / Right
   - The vehicle will change lanes and then follow longitudinal
     behavior for the "KL" state in the new lane. */

  this->available_states = {"KL"};
  if (this->d > 4 && !car_to_left) {
    this->available_states.push_back("LCL");
  }
  if (this->d < 8 && !car_to_right) {
    this->available_states.push_back("LCR");
  }
}

vector<vector<double>> Vehicle::get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_just_ahead) {
  // Returns two lists s_target and d_target in a single vector - s_target includes 
  // [s, s_dot, and s_ddot] and d_target includes the same
  // If no leading car found target lane, ego car will make up PERCENT_V_DIFF_TO_MAKE_UP of the difference
  // between current velocity and target velocity. If leading car is found set target s to FOLLOW_DISTANCE
  // and target s_dot to leading car's s_dot based on predictions
  int target_lane, current_lane = this->d / 4; 
  double target_d; 
  // **** TARGETS ****
  // lateral displacement : depends on state
  // lateral velocity : 0
  double target_d_d = 0;
  // lateral acceleration : 0
  double target_d_dd = 0;
  // longitudinal velocity : current velocity + max allowed accel * duration
  double target_s_d = min(this->s_d + MAX_INSTANTANEOUS_ACCEL/4 * duration, SPEED_LIMIT);
  target_s_d = SPEED_LIMIT;  
  // longitudinal acceleration : zero ?
  double target_s_dd = 0;
  // longitudinal acceleration : difference between current/target velocity over trajectory duration?
  //double target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);
  // longitudinal displacement : current displacement plus difference in current/target velocity times 
  // trajectory duration
  double target_s = this->s + (this->s_d + target_s_d) / 2 * duration;

  vector<double> leading_vehicle_s_and_sdot;

  if(state.compare("KL") == 0)
  {
    target_d = (double)current_lane * 4 + 2;
    target_lane = target_d / 4;
  }
  else if(state.compare("LCL") == 0)
  {
    target_d = ((double)current_lane - 1) * 4 + 2;
    target_lane = target_d / 4;
  }
  else if(state.compare("LCR") == 0)
  {
    target_d = ((double)current_lane + 1) * 4 + 2;
    target_lane = target_d / 4;
  }
  
  // replace target_s variables if there is a leading vehicle close enough
  leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, predictions, duration);
  double leading_vehicle_s = leading_vehicle_s_and_sdot[0];
  if (leading_vehicle_s - target_s < FOLLOW_DISTANCE && leading_vehicle_s > this->s) {

    target_s_d = leading_vehicle_s_and_sdot[1];

    if (fabs(leading_vehicle_s - target_s) < 0.5 * FOLLOW_DISTANCE) {
      //cout << "TOO CLOSE IN LANE " << target_lane << "!! current target speed: " << target_s_d;
      target_s_d -= 1; // slow down if too close
      //cout << "  new target speed: " << target_s_d << endl;
    }

    target_s = leading_vehicle_s - FOLLOW_DISTANCE;
    // target acceleration = difference between start/end velocities over time duration? or just zero?
    //target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);

    // // DEBUG
    // cout << "NEARBY LEAD VEHICLE DETECTED!  ";
    // cout << "s: " << leading_vehicle_s_and_sdot[0]
    //    << ", lane: " << target_lane 
    //    << ", speed: " << leading_vehicle_s_and_sdot[1] << endl;
  }

  // emergency brake
  if (car_just_ahead) {
    target_s_d = 0.0;
  }

  return {{target_s, target_s_d, target_s_dd}, {target_d, target_d_d, target_d_dd}};
}

vector<double> Vehicle::get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions, double duration) {
  // returns s and s_dot for the nearest (ahead) vehicle in target lane
  // this assumes the dummy vehicle will keep its lane and velocity, it will return the end position
  // and velocity (based on difference between last two positions)
  double nearest_leading_vehicle_speed = 0, nearest_leading_vehicle_distance = 99999;
  for (auto prediction : predictions) {
    vector<vector<double>> pred_traj = prediction.second;
    int pred_lane = pred_traj[0][1] / 4;
    if (pred_lane == target_lane) {
      double start_s = pred_traj[0][0];
      double predicted_end_s = pred_traj[pred_traj.size()-1][0];
      double next_to_last_s = pred_traj[pred_traj.size()-2][0];
      double dt = duration / N_SAMPLES;
      double predicted_s_dot = (predicted_end_s - next_to_last_s) / dt;
      if (predicted_end_s < nearest_leading_vehicle_distance && start_s > this->s) {
        nearest_leading_vehicle_distance = predicted_end_s;
        nearest_leading_vehicle_speed = predicted_s_dot;
      }
    }
  }
  return {nearest_leading_vehicle_distance, nearest_leading_vehicle_speed};
}

vector<vector<double>> Vehicle::perturb(vector<vector<double>> target_s_and_d) {
  // randomly perturb the target of the trajectory 
  double perturbed_s, perturbed_s_dot, perturbed_s_ddot,
       perturbed_d, perturbed_d_dot, perturbed_d_ddot;
  // pull out the individual targets
  vector<double> target_s_vars = target_s_and_d[0];
  vector<double> target_d_vars = target_s_and_d[1];
  double target_s = target_s_vars[0];
  double target_s_dot = target_s_vars[1];
  double target_s_ddot = target_s_vars[2];
  double target_d = target_d_vars[0];
  double target_d_dot = target_d_vars[1];
  double target_d_ddot = target_d_vars[2];

  random_device rd;
  mt19937 e2(rd());
  normal_distribution<> nd1(target_s, SIGMA_S);
  perturbed_s = nd1(e2);
  normal_distribution<> nd2(target_s_dot, SIGMA_S_DOT);
  perturbed_s_dot = nd2(e2);
  normal_distribution<> nd3(target_s_ddot, SIGMA_S_DDOT);
  perturbed_s_ddot = nd3(e2);
  normal_distribution<> nd4(target_d, SIGMA_D);
  perturbed_d = nd4(e2);
  normal_distribution<> nd5(target_d_dot, SIGMA_D_DOT);
  perturbed_d_dot = nd5(e2);
  normal_distribution<> nd6(target_d_ddot, SIGMA_D_DDOT);
  perturbed_d_ddot = nd6(e2);

  return {{perturbed_s, perturbed_s_dot, perturbed_s_ddot},
      {perturbed_d, perturbed_d_dot, perturbed_d_ddot}};
}

vector<vector<double>> Vehicle::generate_traj_for_target(vector<vector<double>> target, double duration) {
  // takes a target {{s, s_dot, s_ddot}, {d, d_dot, d_ddot}} and returns a Jerk-Minimized Trajectory
  // (JMT) connecting current state (s and d) to target state in a list of s points and a list of d points
  // ex. {{s1, s2, ... , sn}, {d1, d2, ... , dn}}
  vector<double> target_s = target[0];
  vector<double> target_d = target[1];
  vector<double> current_s = {this->s, this->s_d, this->s_dd};
  vector<double> current_d = {this->d, this->d_d, this->d_dd};

  // determine coefficients of optimal JMT 
  this->s_traj_coeffs = get_traj_coeffs(current_s, target_s, duration);
  this->d_traj_coeffs = get_traj_coeffs(current_d, target_d, duration);

  // // DEBUG
  // cout << "s coeffs: ";
  // for (auto s : this->s_traj_coeffs) cout << s << ",";
  // cout << endl;
  // cout << "d coeffs: ";
  // for (auto d : this->d_traj_coeffs) cout << d << ",";
  // cout << endl << endl;

  vector<double> s_traj;
  vector<double> d_traj;

  // populate s and t trajectories at each time step
  for (int i = 0; i < N_SAMPLES; i++) {
    double t = i * duration/N_SAMPLES;
    double s_val = 0, d_val = 0;
    for (int j = 0; j < s_traj_coeffs.size(); j++) {
      s_val += this->s_traj_coeffs[j] * pow(t, j);
      d_val += this->d_traj_coeffs[j] * pow(t, j);
    }
    s_traj.push_back(s_val);
    d_traj.push_back(d_val);
  }

  return {s_traj, d_traj};
}

vector<double> Vehicle::differentiate_coeffs(vector<double> coeffs) {
  vector<double> diff_coeffs;
  for (int i = 1; i < coeffs.size(); i++) {
    diff_coeffs.push_back(i * coeffs[i]);
  }
  return diff_coeffs;
}

double Vehicle::evaluate_coeffs_at_time(vector<double> coeffs, double time) {
  double eval = 0;
  for (int i = 0; i < coeffs.size(); i++) {
    eval += coeffs[i] * pow(time, i);
  }
  return eval;
}

vector<vector<double>> Vehicle::generate_predictions(double traj_start_time, double duration) {

  // Generates a list of predicted s and d positions for dummy constant-speed vehicles
  // Because ego car trajectory is considered from end of previous path, we should also consider the 
  // trajectories of other cars starting at that time.

  vector<vector<double>> predictions;
  for( int i = 0; i < N_SAMPLES; i++)
  {
    double t = traj_start_time + (i * duration/N_SAMPLES);
    double new_s = this->s + this->s_d * t;
    vector<double> s_and_d = {new_s, this->d};
    predictions.push_back(s_and_d);
  }
  return predictions;
}

string Vehicle::display() {

  ostringstream oss;
  
  oss << "s:  "    << this->s    << "\n";
  oss << "s_d:  "  << this->s_d  << "\n";
  oss << "s_dd:  " << this->s_dd << "\n";
  oss << "d:  "    << this->d    << "\n";
  oss << "d_d:  "  << this->d_d  << "\n";
  oss << "d_dd:  " << this->d_dd << "\n";
  
  return oss.str();
}