#ifndef CONSTANTS
#define CONSTANTS

#define VEHICLE_RADIUS 1.5              // meters
#define FOLLOW_DISTANCE 6.0             // distance to keep behind leading cars
#define PERCENT_V_DIFF_TO_MAKE_UP 0.5   // the percent difference between current velocity and target velocity to allow ego car to make up in a single trajectory      

#define PREVIOUS_PATH_POINTS_TO_KEEP 10
#define NUM_PATH_POINTS 150

#define TRACK_LENGTH 6945.554           // meters

// number of waypoints to use for interpolation
#define NUM_WAYPOINTS_BEHIND 5
#define NUM_WAYPOINTS_AHEAD 5

#define N_SAMPLES 150
#define DT 0.02                         // seconds

#define NUM_RANDOM_TRAJ_TO_GEN 10       // the number of perturbed trajectories to generate (for each duration)
#define NUM_TIMESTEPS_TO_PERTURB 5      // the number of timesteps, +/- target time, to perturb trajectories

// sigma values for perturbing targets
#define SIGMA_S 5.0                    // s
#define SIGMA_S_DOT 4.0                 // s_dot
#define SIGMA_S_DDOT 2.0                // s
#define SIGMA_D 1.0                     // d
#define SIGMA_D_DOT 1.0                 // d_dot
#define SIGMA_D_DDOT 1.0                // d_double_dot
#define SIGMA_T 2.0

#define MAX_INSTANTANEOUS_JERK 10       // m/s/s/s
#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2      // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1       // m/s

#define SPEED_LIMIT 22                  // m/s

// cost function weights
#define TIME_DIFF_COST_WEIGHT 10
#define TRAJ_DIFF_COST_WEIGHT 10
#define COLLISION_COST_WEIGHT 9999
#define BUFFER_COST_WEIGHT 10
#define SPEED_LIMIT_COST_WEIGHT 1000
#define EFFICIENCY_COST_WEIGHT 100
#define MAX_ACCEL_COST_WEIGHT 1000
#define AVG_ACCEL_COST_WEIGHT 100
#define MAX_JERK_COST_WEIGHT 1000
#define AVG_JERK_COST_WEIGHT 100

#endif