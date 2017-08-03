#include "vehicle.h"

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
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(map<int, vector<vector<double>> > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "d": 0},
        {"s" : 6, "d": 0},
        {"s" : 8, "d": 0},
        {"s" : 10, "d": 0},
      ]
    }

    */
    //state = "KL"; // this is an example of how you change state.
    vector<string> states = {"KL"};
    if (this->d > 4) {
        states.push_back("LCL");
    }
    if (this->d < 8) {
        states.push_back("LCR");
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
	
	oss << "s:    " << this->s << "\n";
    oss << "d:    " << this->d << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

vector<double> Vehicle::state_at(double t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->d, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, double at_time) {

	/*
    Simple collision detection.
    */
    vector<double> check1 = state_at(at_time);
    vector<double> check2 = other.state_at(at_time);
    return (abs(check1[0]-check2[0]) <= L && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
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

vector<vector<double> > Vehicle::generate_predictions(int horizon = 10) {

    vector<vector<double> > predictions;
    for( int i = 0; i < horizon; i++)
    {
        double at_time = i * 0.2
        vector<double> check1 = state_at(at_time);
        vector<double> d_and_s = {check1[0], check1[1]};
        predictions.push_back(d_and_s);
    }
    return predictions;

}