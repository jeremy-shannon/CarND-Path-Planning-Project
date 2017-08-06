#include "vehicle.h"
#include "constants.h"
#include "costs.h"

#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

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
        vector<vector<double>> target_s_and_d = get_target_for_state(state);
        double target_time = N_SAMPLES * DT;

        // Perturb trajectories, first by duration
        for (int i = -5; i < 6; i++) {
            current_time = target_time + (i * DT);
            // Perturb by sigma s and d values
            for (int i = 0; i < NUM_RANDOM_TRAJ_TO_GEN; i++) {
                vector<vector<double>> possible_traj = generate_traj_for_target(target_s_and_d);
                double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions, target_s_and_d[0], target_s_and_d[1], target_time, current_time)

                if (current_cost < best_cost) {
                    best_cost = current_cost;
                    best_frenet_traj = possible_traj;
                }
            }
        }
        
    }

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

    vector<double> costs;
    double cost;
    for (string test_state : states) {
        cost = 0;
        // create copy of our vehicle
        Vehicle test_v = Vehicle(this->d, this->s, this->v, this->a);
        test_v.state = test_state;
        test_v.realize_state(predictions);
        vector<int> test_v_state = test_v.state_at(1);
        int pred_d = test_v_state[0];
        int pred_s = test_v_state[1];
        int pred_v = test_v_state[2];
        int pred_a = test_v_state[3];
        //cout << "pred lane: " << pred_lane << " s: " << pred_s << " v: " << pred_v << " a: " << pred_a << endl;

        cost += 1*(10 - pred_v);
        cost += 1*(pow(3 - pred_d, 2));
        cost += 10*(1 - exp(-abs(pred_d - 3)/(300 - (double)pred_s)));
        if (pred_lane < 0 || pred_d > 3) {
            cost += 1000;
        }
        
        cout << "cost: " << cost << endl;
        costs.push_back(cost);
    }
    double min_cost = 99999;
    int min_cost_index = 0;
    for (int i = 0; i < costs.size(); i++) {
        //cout << "cost[" << i << "]: " << costs[i] << endl;
        if (costs[i] < min_cost) {
            min_cost = costs[i];
            min_cost_index = i;
            
        }
    }
    
    state = states[min_cost_index];
    //state = "LCR";
    cout << "chosen state: " << state << endl;


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

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }

}

void Vehicle::realize_keep_lane(map<int,vector< vector<double> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->d, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<double> > > predictions, string direction) {
	
    if (direction.compare("R") == 0)
    {
    	this->d = this->d - 1;
    } else {
        this->d++;
    }
    
    int d = this->d;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, d, s);
}

vector<vector<double>> Vehicle::generate_predictions() {

    // Generates a list of predicted s and d positions for dummy constant-speed vehicles

    vector<vector<double>> predictions;
    for( int i = 0; i < N_SAMPLES; i++)
    {
        double at_time = i * DT
        vector<double> check1 = state_at(at_time);
        vector<double> d_and_s = {check1[0], check1[1]};
        predictions.push_back(d_and_s);
    }
    return predictions;

}