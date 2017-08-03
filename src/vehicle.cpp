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
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
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
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
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
        Vehicle test_v = Vehicle(this->lane, this->s, this->v, this->a);
        test_v.state = test_state;
        test_v.realize_state(predictions);
        vector<int> test_v_state = test_v.state_at(1);
        int pred_lane = test_v_state[0];
        int pred_s = test_v_state[1];
        int pred_v = test_v_state[2];
        int pred_a = test_v_state[3];
        //cout << "pred lane: " << pred_lane << " s: " << pred_s << " v: " << pred_v << " a: " << pred_a << endl;
        
        cout << "tested state: " << test_state << endl;
        
        // check for collisions
        map<int, vector<vector<int> > >::iterator it = predictions.begin();
        vector<vector<vector<int> > > in_front;
        while(it != predictions.end())
        {
            int index = it->first;
            vector<vector<int> > v = it->second;
            if ((v[1][0] == pred_lane) && (abs(v[1][1] - pred_s) <= L) && index != -1) {
                cout << "coll w/ car: " << index << ", "
                     << v[1][0] << " " << pred_lane << ", " 
                     << v[1][1] << " " << pred_s << endl;
                cost += 1000;
            }
            it++;
        }
        
        cost += 1*(10 - pred_v);
        cost += 1*(pow(3 - pred_lane, 2));
        cost += 10*(1 - exp(-abs(pred_lane - 3)/(300 - (double)pred_s)));
        if (pred_lane < 0 || pred_lane > 3) {
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

void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    max_acceleration = road_data[2];
    goal_lane = road_data[3];
    goal_s = road_data[4];
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "d:    " << this->d << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
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

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	
    if (direction.compare("R") == 0)
    {
    	this->lane = this->lane - 1;
    } else {
        this->lane++;
    }
    
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

vector<vector<double> > Vehicle::generate_predictions(int horizon = 10) {

    vector<vector<double> > predictions;
    for( int i = 0; i < horizon; i++)
    {
        double
        vector<double> check1 = state_at(i);
        vector<double> d_and_s = {check1[0], check1[1]};
        predictions.push_back(d_and_s);
    }
    return predictions;

}