#include "vehicle.h"
#include "constants.h"
#include "costs.h"
#include "jmt.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <random>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd) {

    this->s    = s;             // s position
    this->s_d  = s_d;           // s dot - velocity in s
    this->s_dd = s_dd;          // s dot-dot - acceleration in s
    this->d    = d;             // d position
    this->d_d  = d_d;           // d dot - velocity in d
    this->d_dd = d_dd;          // d dot-dot - acceleration in d
    state = "CS";

}

Vehicle::~Vehicle() {}

vector<vector<double>> Vehicle::get_best_frenet_trajectory(map<int, vector<vector<double>>> predictions) {
    update_available_states();
    vector<vector<double>> best_frenet_traj;
    double best_cost = 999999;
    for (string state : available_states) {

        // target state (s and d) and time
        vector<vector<double>> target_s_and_d = get_target_for_state(state, predictions);
        double target_time = N_SAMPLES * DT;

        // Perturb trajectories, first by duration
        for (int i = -NUM_TIMESTEPS_TO_PERTURB; i <= NUM_TIMESTEPS_TO_PERTURB; i++) {
            double perturbed_time = target_time + (i * DT);

            // Perturb by sigma s and d values
            for (int i = 0; i < NUM_RANDOM_TRAJ_TO_GEN; i++) {
                vector<vector<double>> perturbed_target = perturb(target_s_and_d);

                vector<vector<double>> possible_traj = generate_traj_for_target(perturbed_target);

                double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions, target_s_and_d[0], target_s_and_d[1], target_time, perturbed_time);

                if (current_cost < best_cost) {
                    best_cost = current_cost;
                    best_frenet_traj = possible_traj;
                }
            }
        }
        
    }
    return best_frenet_traj;
}

void Vehicle::update_available_states() {
	/*  Updates the available "states" based on the current state:
    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.
    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane. */

    this->available_states = {"KL"};
    if (this->d > 4) {
        this->available_states.push_back("LCL");
    }
    if (this->d < 8) {
        this->available_states.push_back("LCR");
    }
}

vector<vector<double>> Vehicle::get_target_for_state(string state, map<int, vector<vector<double>>> predictions) {
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
    // longitudinal velocity : current velocity + difference between current and limit * percent allowed to 
    // make up
    double target_s_d = this->s_d + (SPEED_LIMIT - this->s_d) * PERCENT_V_DIFF_TO_MAKE_UP;
    // longitudinal acceleration : zero ?
    double target_s_dd = 0;
    // longitudinal acceleration : difference between current/target velocity over trajectory duration?
    //double target_s_dd = (target_s_d - this->s_d) / (N_SAMPLES * DT);
    // longitudinal displacement : current displacement plus difference in current/target velocity times 
    // trajectory duration
    double target_s = this->s + (target_s_d + this->s_d) / 2 * (N_SAMPLES * DT);

    vector<double> leading_vehicle_s_and_sdot;

    if(state.compare("KL") == 0)
    {
        target_d = (double)current_lane * 4 + 2;
        target_lane = target_d / 4;
    	leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, predictions);
    }
    else if(state.compare("LCL") == 0)
    {
        target_d = ((double)current_lane - 1) * 4 + 2;
        target_lane = target_d / 4;
    	leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, predictions);
    }
    else if(state.compare("LCR") == 0)
    {
        target_d = ((double)current_lane + 1) * 4 + 2;
        target_lane = target_d / 4;
    	leading_vehicle_s_and_sdot = get_leading_vehicle_data_for_lane(target_lane, predictions);
    }

    // replace target_s variables if there is a leading vehicle close enough
    if (leading_vehicle_s_and_sdot[0] < target_s) {
        target_s = leading_vehicle_s_and_sdot[0] - FOLLOW_DISTANCE;
        target_s_d = leading_vehicle_s_and_sdot[1];
        // target acceleration = difference between start/end velocities over time duration? or just zero?
        // target_s_dd = (target_s_d - this->s_d) / (NUMBER_SAMPLE * DT);
    }

    return {{target_s, target_s_d, target_s_dd}, {target_d, target_d_d, target_d_dd}};
}

vector<double> Vehicle::get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions) {
    // returns vehicle s and s_dot
    // this assumes the dummy vehicle will keep its lane and velocity, it will return the end position
    // and velocity (based on difference between last two positions)
    double leading_vehicle_speed = 0, leading_vehicle_distance = 99999;
    for (auto prediction : predictions) {
        vector<vector<double>> pred_traj = prediction.second;
        if (pred_traj[0][1] == target_lane) {
            double predicted_end_s = pred_traj[pred_traj.size()-1][0];
            double next_to_last_s = pred_traj[pred_traj.size()-2][0];
            double predicted_s_dot = (predicted_end_s - next_to_last_s) / DT;
            if (predicted_end_s < leading_vehicle_distance) {
                leading_vehicle_distance = predicted_end_s;
                leading_vehicle_speed = predicted_s_dot;
            }
        }
    }
    return {leading_vehicle_distance, leading_vehicle_speed};
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

vector<vector<double>> Vehicle::generate_traj_for_target(vector<vector<double>> target) {
    // takes a target {{s, s_dot, s_ddot}, {d, d_dot, d_ddot}} and returns a Jerk-Minimized Trajectory
    // (JMT) connecting current state (s and d) to target state in a list of s points and a list of d points
    // ex. {{s1, s2, ... , sn}, {d1, d2, ... , dn}}
    vector<double> target_s = target[0];
    vector<double> target_d = target[1];
    vector<double> current_s = {this->s, this->s_d, this->s_dd};
    vector<double> current_d = {this->d, this->d_d, this->d_dd};

    double duration = N_SAMPLES * DT;

    // determine coefficients of optimal JMT 
    vector<double> s_traj_coeffs = get_traj_coeffs(current_s, target_s, duration);
    vector<double> d_traj_coeffs = get_traj_coeffs(current_d, target_d, duration);

    vector<double> s_traj;
    vector<double> d_traj;

    // populate s and t trajectories at each time step
    for (int i = 0; i < N_SAMPLES; i++) {
        double t = i * DT;
        double s_val = 0, d_val = 0;
        for (int j = 0; j < s_traj_coeffs.size(); i++) {
            s_val += s_traj_coeffs[j] * pow(t, i);
            d_val += d_traj_coeffs[j] * pow(t, i);
        }
        s_traj.push_back(s_val);
        d_traj.push_back(d_val);
    }

    return {s_traj, d_traj};
}

vector<vector<double>> Vehicle::generate_predictions() {

    // Generates a list of predicted s and d positions for dummy constant-speed vehicles

    vector<vector<double>> predictions;
    for( int i = 0; i < N_SAMPLES; i++)
    {
        double new_s = this->s + this->s_d * i * DT;
        vector<double> s_and_d = {new_s, this->d};
        predictions.push_back(s_and_d);
    }
    return predictions;
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    "    << this->s    << "\n";
	oss << "s_d:    "  << this->s_d  << "\n";
	oss << "s_dd:    " << this->s_dd << "\n";
    oss << "d:    "    << this->d    << "\n";
    oss << "d_d:    "  << this->d_d  << "\n";
    oss << "d_dd:    " << this->d_dd << "\n";
    
    return oss.str();
}