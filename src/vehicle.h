#ifndef VEHICLE
#define VEHICLE

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  struct collider{

    bool collision ; // is there a collision?
    double time; // time collision happens

  };

  double L = 1.7;  // vehicle "size"

  double s;
  double s_d;
  double s_dd;
  double d;
  double d_d;
  double d_dd;
  string state;

  /**
  * Constructor
  */
Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(map<int, vector<vector<double>> > predictions);

  string display();

  void increment(int dt);

  vector<double> state_at(double t);

  bool collides_with(Vehicle other, double at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector<vector<double>> > predictions);

  void realize_constant_speed();

  void realize_keep_lane(map<int, vector<vector<double>> > predictions);

  void realize_lane_change(map<int, vector<vector<double>> > predictions, string direction);

  vector<vector<double>> generate_predictions(int horizon);

};

#endif