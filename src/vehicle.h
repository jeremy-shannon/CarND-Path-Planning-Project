#ifndef VEHICLE
#define VEHICLE

#include <vector>
#include <map>
#include <string>

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
  vector<double> s_traj_coeffs, d_traj_coeffs;

  /**
  * Constructors
  */
  Vehicle();
  Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  // DEPRECATED
  vector<vector<double>> get_best_frenet_trajectory(map<int, vector<vector<double>>> predictions, double duration);

  void update_available_states(bool car_to_left, bool car_to_right);

  vector<vector<double>> get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration, bool car_just_ahead);

  vector<double> get_leading_vehicle_data_for_lane(int target_lane, map<int, vector<vector<double>>> predictions, double duration);

  // DEPRECATED
  vector<vector<double>> perturb(vector<vector<double>> target_s_and_d);

  vector<vector<double>> generate_traj_for_target(vector<vector<double>> perturbed_target, double duration);

  vector<double> differentiate_coeffs(vector<double> coeffs); 

  double evaluate_coeffs_at_time(vector<double> coeffs, double time);

  vector<vector<double>> generate_predictions(double traj_start_time, double duration);

  string display();

};

#endif