#ifndef SPLINE_HPP_
#define SPLINE_HPP_


#include <Eigen/Eigen> // AFTER GRIDMAP!
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>


/**
 * @brief The spline is used to interpolate gauge force values
 */
class SplineFunction {

public:
  SplineFunction();

  /**
   * @brief Construct a new Spline Function Interpolation for gauge force values
   *
   * @param x_vec reference gauge readings
   * @param y_vec reference force (N) values
   */
  SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec);

  /**
   * @brief Interpolate force from reading
   *
   * @param x reading (value)
   * @return double interpolated force (N)
   */
  double interp(double x) const;
  float  interpf(float x) const;

private:
  double scaled_value(double x) const;

  Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const;

  double x_min;
  double x_max;
  double y_min;
  double y_max;

  // Spline of one-dimensional "points."
  Eigen::Spline<double, 1> spline_;
};

#endif //SPLINE_HPP_