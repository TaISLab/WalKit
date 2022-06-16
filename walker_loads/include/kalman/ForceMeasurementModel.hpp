#ifndef KALMAN_EXAMPLES_LEG_FORCEMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_LEG_FORCEMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include <cmath>

namespace KalmanExamples
{
namespace Step
{

/**
 * @brief Measurement vector measuring leg position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class ForceMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(ForceMeasurement, T, 1)
    
    //! current measured force difference
    static constexpr size_t DF = 0;
        
    T  df() const { return (*this)[ DF ]; }
    T& df()       { return (*this)[ DF ]; }
};

/**
 * @brief Force measurement
 *
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class ForceMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, ForceMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  KalmanExamples::Step::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::Step::ForceMeasurement<T> FM;
    
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     * 
     * h(x) = df = f0 + f1 * sin(w*t + phi + d) = f0 + f1 * sin(vp + d)
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    FM h(const S& x) const
    {
        FM measurement;        
        measurement.df() = x.f0() + x.f1() * std::sin(x.vp() + x.d());
      
        return measurement;
    }

protected:
    
    static constexpr T dosPi = 2.0 * M_PI;
    
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize 
     */
    void updateJacobians( const S& x )
    {
        // h(x) = df = f0 + f1 * sin(vp + d)
        //   s  = [v0 v1 f0 f1 w d  vp] 
        // H = d/ds * h(s) = [dh/dv0, dh/dv1, dh/df0, dh/df1, dh/dw, dh/dd, dh/dvp] (Jacobian of measurement function w.r.t. the state)
        this->H.setZero();
        
        this->H( FM::DF, S::V0 ) = 0;
        this->H( FM::DF, S::V1 ) = 0;
        this->H( FM::DF, S::F0 ) = 1;
        this->H( FM::DF, S::F1 ) = std::sin(x.vp()+ x.d());
        this->H( FM::DF, S::W  ) = 0;  //  x.f1() * std::cos(x.vp()+ x.d());  //  should be 0
        this->H( FM::DF, S::D  ) = x.f1() * std::cos(x.vp()+ x.d());
        this->H( FM::DF, S::VP ) = x.f1() * std::cos(x.vp()+ x.d());

    }    

};

} // namespace Step
} // namespace KalmanExamples

#endif