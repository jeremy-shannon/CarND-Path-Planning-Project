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

  double preferred_buffer = 6; // impacts "keep lane" behavior.

  double d;

  double s;

  double v;

  double a;

  double target_speed;

  int lanes_available;

  double max_acceleration;

  string state;

  /**
  * Constructor
  */
  Vehicle(double d, double s, double v, double a);

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

  int _max_accel_for_lane(map<int, vector<vector<double>> > predictions, double d, double s);

  void realize_keep_lane(map<int, vector<vector<double>> > predictions);

  void realize_lane_change(map<int, vector<vector<double>> > predictions, string direction);

  vector<vector<double>> generate_predictions(int horizon);

};

#endif