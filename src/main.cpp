#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "smoother.h"
#include "constants.h"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2( (map_y-y),(map_x-x) );
	double angle = abs(theta-heading);
	if(angle > pi()/4)
	{
		closestWaypoint++;
	}
	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];
	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];
					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];
					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// DEBUG
					cout << endl << "**************** ITERATION BEGIN ****************" << endl << endl;

					// ********************* CONSTRUCT INTERPOLATED WAYPOINTS OF NEARBY AREA **********************
					int num_waypoints = map_waypoints_x.size();
					int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
					vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y;
					for (int i = -NUM_WAYPOINTS_BEHIND; i < NUM_WAYPOINTS_AHEAD; i++) {
						// for smooting, take so many previous and so many subsequent waypoints
						int idx = (next_waypoint_index+i) % num_waypoints;
						if (idx < 0) {
							idx += num_waypoints;
						}
						coarse_waypoints_s.push_back(map_waypoints_s[idx]);
						coarse_waypoints_x.push_back(map_waypoints_x[idx]);
						coarse_waypoints_y.push_back(map_waypoints_y[idx]);
					}
					// correct for wrap in s values
					for (int i = 1; i < coarse_waypoints_s.size(); i++) {
						if (coarse_waypoints_s[i] < coarse_waypoints_s[i-1]) {
							coarse_waypoints_s[i] += TRACK_LENGTH;
						}
					}

					// DEBUG
					cout << "****WAYPOINT INTERPOLATION****" << endl;
					cout << "coarse s: ";
					for (auto s: coarse_waypoints_s) cout << s << ", ";
					cout << endl;
					cout << "coarse x: ";
					for (auto x: coarse_waypoints_x) cout << x << ", ";
					cout << endl;
					cout << "coarse y: ";
					for (auto y: coarse_waypoints_y) cout << y << ", ";
					cout << endl;

					// interpolation parameters, 240 points at 0.5 distance increment = 120 m = 4 30-m sections
					double dist_inc = 1.0;	
					int num_interpolation_points = 275;
					vector<double> interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y;
					// interpolated s is simply...
					interpolated_waypoints_s.push_back(coarse_waypoints_s[0]);
					for (int i = 1; i < num_interpolation_points; i++) {
						interpolated_waypoints_s.push_back(coarse_waypoints_s[0] + i * dist_inc);
					}
					interpolated_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, dist_inc, num_interpolation_points);
					interpolated_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, dist_inc, num_interpolation_points);

					// DEBUG
					cout << "interp s: ";
					for (int i = 0; i <= num_interpolation_points; i += num_interpolation_points/4-1) {
						cout << "(" << i << ")" << interpolated_waypoints_s[i] << " ";
					}
					cout << endl;
					cout << "interp x: ";
					for (int i = 0; i <= num_interpolation_points; i += num_interpolation_points/4-1) {
						cout << "(" << i << ")" << interpolated_waypoints_x[i] << " ";
					}
					cout << endl;
					cout << "interp y: ";
					for (int i = 0; i <= num_interpolation_points; i += num_interpolation_points/4-1) { 
						cout << "(" << i << ")" << interpolated_waypoints_y[i] << " ";
					}
					cout << endl << endl;

					// **************** DETERMINE EGO CAR PARAMETERS AND CONSTRUCT VEHICLE OBJECT ******************
					// Vehicle class requires s,s_d,s_dd,d,d_d,d_dd - in that order
					double pos_s, s_dot, s_ddot;
					double pos_d, d_dot, d_ddot;
					// Other values necessary for determining these based on future points in previous path
					double pos_x, pos_y, angle;

					int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());

					// use default values if not enough previous path points
					if (subpath_size < 4) {
						pos_x = car_x;
						pos_y = car_y;
						angle = deg2rad(car_yaw);
						pos_s = car_s;
						pos_d = car_d;
						s_dot = car_speed;
						d_dot = 0;
						s_ddot = 0;
						d_ddot = 0;
					} else {
						// consider current position to be last point of previous path to be kept
						pos_x = previous_path_x[subpath_size-1];
						pos_y = previous_path_y[subpath_size-1];
						double pos_x2 = previous_path_x[subpath_size-2];
						double pos_y2 = previous_path_y[subpath_size-2];
						angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
						//vector<double> frenet = getFrenet(pos_x, pos_y, angle, interpolated_waypoints_x, interpolated_waypoints_y);
						vector<double> frenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
						pos_s = frenet[0];
						pos_d = frenet[1];

						// have to get another point to calculate s_dot
						double pos_x3 = previous_path_x[subpath_size-3];
						double pos_y3 = previous_path_y[subpath_size-3];
						double angle2 = atan2(pos_y2-pos_y3,pos_x2-pos_x3);
						//frenet = getFrenet(pos_x2, pos_y2, angle2, interpolated_waypoints_x, interpolated_waypoints_y);
						frenet = getFrenet(pos_x2, pos_y2, angle2, map_waypoints_x, map_waypoints_y);
						double pos_s2 = frenet[0];
						double pos_d2 = frenet[1];
						s_dot = (pos_s - pos_s2) / PATH_DT;
						d_dot = (pos_d - pos_d2) / PATH_DT;
						
						// and yet another point to calculate s_ddot
						double pos_x4 = previous_path_x[subpath_size-4];
						double pos_y4 = previous_path_y[subpath_size-4];
						double angle3 = atan2(pos_y3-pos_y4,pos_x3-pos_x4);
						//frenet = getFrenet(pos_x3, pos_y3, angle3, interpolated_waypoints_x, interpolated_waypoints_y);
						frenet = getFrenet(pos_x3, pos_y3, angle3, map_waypoints_x, map_waypoints_y);
						double pos_s3 = frenet[0];
						double pos_d3 = frenet[1];
						double s_d2 = (pos_s2 - pos_s3) / PATH_DT;
						double d_d2 = (pos_d2 - pos_d3) / PATH_DT;
						s_ddot = (s_dot - s_d2) / PATH_DT;
						d_ddot = (d_dot - d_d2) / PATH_DT;
					}		
					
					Vehicle my_car = Vehicle(pos_s, s_dot, s_ddot, pos_d, d_dot, d_ddot);

					// DEBUG
					cout << "****EGO CAR DATA****" << endl;
					cout << "ego state (x,y,s,d,yaw,speed): " << car_x << ", " << car_y << ", " << car_s << ", " << car_d << ", " << car_yaw << ", " << car_speed << endl;
					cout << "planning state (x,y,yaw): " << pos_x << ", " << pos_y << ", " << angle << endl;
					cout << "planning state (s,s_d,s_dd),(d,d_d,d_dd): (" << pos_s << ", " << s_dot << ", " << s_ddot;
					cout << ") (" << pos_d << ", " << d_dot << ", " << d_ddot << ")" << endl << endl;

					// ********************* GENERATE PREDICTIONS FROM SENSOR FUSION DATA **************************
					// The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.
					vector<Vehicle> other_cars;
					map<int, vector<vector<double>>> predictions;
					for (auto sf: sensor_fusion) {
						double other_car_vel = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
						Vehicle other_car = Vehicle(sf[5], other_car_vel, 0, sf[6], 0, 0);
						int v_id = sf[0];
						vector<vector<double>> preds = other_car.generate_predictions();
						predictions[v_id] = preds;
					}

					// // DEBUG
					// cout << "****SENSOR FUSION DATA****" << endl;
					// cout << "sensor fusion: (id, x, y, vx, vy, s, d), (distance from ego)" << endl;
					// for (auto sf: sensor_fusion) {
					// 	cout << "(" << sf[0] << ": " << sf[1] << "," << sf[2] << "," << sf[3] << "," << sf[4] << "," << sf[5] << "," << sf[6] << ") (" << distance(pos_x, pos_y, sf[1], sf[2]) << ")" << endl;
					// }
					// cout << endl << "predictions: (id, (i s1,d1) (i s2,d2) ... (i sn,dn) - spaced out)" << endl;
					// for (auto pred : predictions) {
					// 	cout << pred.first << " ";
					// 	auto sd = pred.second;
    			// 	for (int i = 0; i < N_SAMPLES; i += N_SAMPLES/3-1) {
					// 		cout << "(" << i << " " << sd[i][0] << "," << sd[i][1] << ") ";
					// 	}
					// 	cout << endl;
					// }
					// cout << endl;

					// ******************************* DETERMINE BEST TRAJECTORY ***********************************
					// where the magic happens?
					// trajectories come back in a list of s values and a list of d values (not zipped together)
					vector<vector<double>> frenet_traj = my_car.get_best_frenet_trajectory(predictions);
					vector<double> traj_xy_point, best_x_traj, best_y_traj, interpolated_x_traj, interpolated_y_traj;

					// // DEBUG
					// cout << "first 5 frenet traj pts: ";
					// for (int i = 0; i < 5; i++) {
					// 	cout << "(" << frenet_traj[0][i] << "," << frenet_traj[1][i] << ") ";
					// }
					// cout << endl << endl;
				

					// ********************* CONVERT, UPSAMPLE, AND PRODUCE NEW PATH ***********************
					// convert points from frenet trajectory to xy
					for (int i = 0; i < N_SAMPLES; i++) {
						//traj_xy_point = getXY(frenet_traj[0][i], frenet_traj[1][i], interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
						traj_xy_point = getXY(frenet_traj[0][i], frenet_traj[1][i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
						best_x_traj.push_back(traj_xy_point[0]);
						best_y_traj.push_back(traj_xy_point[1]);
					}

					// interpolate to upsample x and y trajectories
					vector<double> coarse_times;
					for (int i = 0; i < N_SAMPLES; i++) {
						coarse_times.push_back(i*DT);
					}
					interpolated_x_traj = interpolate_points(coarse_times, best_x_traj, PATH_DT, (NUM_PATH_POINTS - subpath_size));
					interpolated_y_traj = interpolate_points(coarse_times, best_y_traj, PATH_DT, (NUM_PATH_POINTS - subpath_size));

					// add previous path, if any, to next path
					for(int i = 1; i < subpath_size; i++) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					} 
					// add xy points from newly generated path
					for (int i = 1; i < (NUM_PATH_POINTS - subpath_size); i++) {
						//if (subpath_size == 0 && i == 0) continue; // maybe skip start position as a path point?
						next_x_vals.push_back(interpolated_x_traj[i]);
						next_y_vals.push_back(interpolated_y_traj[i]);
					} 

					// DEBUG
					cout << "****TRAJECTORY DATA****" << endl;
					cout << "xy trajectory (spaced-out; i: x,y):" << endl;
					for (int i = 0; i < N_SAMPLES; i += N_SAMPLES/3-1) {
						cout << "(" << i << ": " << best_x_traj[i] << "," << best_y_traj[i] << ") ";
					}
					cout << endl << endl;


					/********************* simple, drive straight example *********************
					double dist_incr = 0.5;
					for(int i = 0; i < 50; i++) {
						next_x_vals.push_back(car_x+(dist_incr*i)*cos(deg2rad(car_yaw)));
						next_y_vals.push_back(car_y+(dist_incr*i)*sin(deg2rad(car_yaw)));
					}***************************************************************************/
					/************************ drive in circles example ************************
					double dist_incr = 0.5;
					for(int i = 0; i < 50-subpath_size; i++) {    
						next_x_vals.push_back(pos_x+(dist_incr)*cos(angle+(i+1)*(pi()/100)));
						next_y_vals.push_back(pos_y+(dist_incr)*sin(angle+(i+1)*(pi()/100)));
						pos_x += (dist_incr)*cos(angle+(i+1)*(pi()/100));
						pos_y += (dist_incr)*sin(angle+(i+1)*(pi()/100));
					}***************************************************************************/
					/***************** drive along interpolated waypoints example ****************
					// get next waypoint from current car position
					int next_waypoint_index_interpolated = NextWaypoint(pos_x, pos_y, angle, 																	interpolated_waypoints_x, interpolated_waypoints_y);
					for (int i = 0; i < 50 - subpath_size; i ++) {
						next_x_vals.push_back(interpolated_waypoints_x[next_waypoint_index_interpolated+i]);
						next_y_vals.push_back(interpolated_waypoints_y[next_waypoint_index_interpolated+i]);
					}******************************************************************************/

					// DEBUG
					cout << "full path (x,y):  \tprevious path (x,y):" << endl;
					for (int i = 0; i < next_x_vals.size(); i++) {
						cout << next_x_vals[i] << ", " << next_y_vals[i];
						if (i < previous_path_x.size()) {
							cout << "  \t" << previous_path_x[i] << ", " << previous_path_y[i];
							if (i == PREVIOUS_PATH_POINTS_TO_KEEP-1) {
								cout << "\tEND OF KEPT PREVIOUS PATH POINTS";
							}
						} 
						cout << endl;
					}
					cout << endl << endl;

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































