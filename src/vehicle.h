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
#include "constants.h"
#include "costs.h"

using namespace std;

class Vehicle {
public:

  double s;
  double s_d;
  double s_dd;
  double d;
  double d_d;
  double d_dd;
  string state;
  vector<string> available_states;
  vector<vector<double>> // ????????? targets?

  /**
  * Constructor
  */
Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<vector<double>> get_best_frenet_trajectory(map<int, vector<vector<double>>> predictions);

  void update_available_states();

  string display();

  void realize_state(map<int, vector<vector<double>>> predictions);

  void realize_keep_lane(map<int, vector<vector<double>>> predictions);

  void realize_lane_change(map<int, vector<vector<double>>> predictions, string direction);

  vector<vector<double>> generate_predictions();

};

#endif