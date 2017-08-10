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

  /**
  * Constructor
  */
  Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<vector<double>> get_best_frenet_trajectory(map<int, vector<vector<double>>> predictions);

  void update_available_states();

  vector<vector<double>> get_target_for_state(string state, map<int, vector<vector<double>>> predictions);

  vector<double> get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions);

  vector<vector<double>> perturb(vector<vector<double>> target_s_and_d);

  vector<vector<double>> generate_traj_for_target(vector<vector<double>> perturbed_target);

  vector<vector<double>> generate_predictions();

  string display();

};

#endif