#include "walker_loads/spline.h"

// The spline is used to interpolate force values
SplineFunction::SplineFunction() {}

SplineFunction::SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec)
    : x_min(x_vec.minCoeff()), 
      x_max(x_vec.maxCoeff()), 
      y_min(y_vec.minCoeff()),
      y_max(y_vec.maxCoeff()),
      // Spline fitting here. X values are scaled down to [0, 1] for this.
      spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate( y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 6), scaled_values( x_vec))) // No more than cubic spline, but accept short vectors.
{}

// x values need to be scaled down in extraction as well.
double SplineFunction::interp(double x) const {
  double y;
  double x_scaled;

  x_scaled = scaled_value(x);

  // interpolation may produce values bigger and lower than our limits ...
  if ( (x_scaled<0) || (x_scaled>1) ){
      y = y_min + ( x_scaled * ( y_max - y_min) );
  } else {
      y = spline_(x_scaled)(0);
  }

  return y;
}

float SplineFunction::interpf(float x) const {
    return (float) interp( (double) x ); 
}

// Helpers to scale X values down to [0, 1]
double SplineFunction::scaled_value(double x) const {
  return (x - x_min) / (x_max - x_min);
}

Eigen::RowVectorXd
SplineFunction::scaled_values(Eigen::VectorXd const &x_vec) const {
  return x_vec.unaryExpr([this](double x) { return scaled_value(x); })
      .transpose();
}