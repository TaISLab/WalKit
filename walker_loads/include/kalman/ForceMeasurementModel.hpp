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
     * h(x) = df = f0 + fa*sin(theta) + fb*cos(theta)
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state. Force and speed share the same theta/w (one gait
     * cycle drives both), but force keeps its own independent (fa,fb) pair:
     * any phase offset between the two signals (previously the explicit `d`
     * state) is implicit in how (fa,fb) relate to (va,vb) at that theta.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    FM h(const S& x) const
    {
        FM measurement;
        measurement.df() = x.f0() + x.fa() * std::sin(x.theta()) + x.fb() * std::cos(x.theta());

        return measurement;
    }

protected:

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
        // h(x) = df = f0 + fa*sin(theta) + fb*cos(theta)
        //   s  = [v0 va vb f0 fa fb w theta]
        // H = d/ds * h(s) (Jacobian of measurement function w.r.t. the state).
        this->H.setZero();

        this->H( FM::DF, S::V0 )    = 0;
        this->H( FM::DF, S::VA )    = 0;
        this->H( FM::DF, S::VB )    = 0;
        this->H( FM::DF, S::F0 )    = 1;
        this->H( FM::DF, S::FA )    = std::sin(x.theta());
        this->H( FM::DF, S::FB )    = std::cos(x.theta());
        this->H( FM::DF, S::W  )    = 0;
        this->H( FM::DF, S::THETA ) = x.fa() * std::cos(x.theta()) - x.fb() * std::sin(x.theta());

    }

};

} // namespace Step
} // namespace KalmanExamples

#endif
