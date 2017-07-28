#ifndef SMOOTHER
#define SMOOTHER

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include "spline.h"

using namespace std;

vector<vector<double>> interpolate_points(vector<double> x_vals, vector<double> y_vals) {

  // uses the spline library to interpolate points connecting a series of x and y values

  vector< vector<double> > output;

  if (x_vals.size() != y_vals.size()) {
    cout << "ERROR! SMOOTHER: interpolate_points size mismatch between x_vals and y_vals" << endl;
    return{ { 0 }, { 0 } };
  }
}



#endif