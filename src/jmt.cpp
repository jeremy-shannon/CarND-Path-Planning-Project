#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

vector<double> get_traj_coeffs(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS
    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Like "start" this is a
        length three array.
    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */

    MatrixXd a(3,3);
    double T2 =  T*T, 
           T3 = T2*T, 
           T4 = T3*T,
           T5 = T4*T;
    a <<  T3,    T4,    T5, 
        3*T2,  4*T3,  5*T4, 
         6*T, 12*T2, 20*T3;
    MatrixXd aInv = a.inverse();
    
    VectorXd b(3);
    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
         end[1] - (           start[1]   +     start[2]*T),
         end[2] - (                            start[2]);
    VectorXd alpha = aInv * b;
    
    vector<double> output = {start[0], start[1], 0.5*start[2], alpha[0], alpha[1], alpha[2]};
    return output;
}

	
