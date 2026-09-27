#ifndef KALMAN_EXAMPLES_LEG_SPEEDMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_LEG_SPEEDMEASUREMENTMODEL_HPP_

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
class SpeedMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(SpeedMeasurement, T, 1)

    //! current measured speed difference
    static constexpr size_t DV = 0;
    T  dv() const { return (*this)[ DV ]; }
    T& dv()       { return (*this)[ DV ]; }
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
class SpeedMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, SpeedMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  KalmanExamples::Step::State<T> S;

    //! Measurement type shortcut definition
    typedef  KalmanExamples::Step::SpeedMeasurement<T> SM;


    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * h(x) = dv = v0 + va*sin(theta) + vb*cos(theta)
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    SM h(const S& x) const
    {
        SM measurement;
        measurement.dv() = x.v0() + x.va() * std::sin(x.theta()) + x.vb() * std::cos(x.theta());

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
        // h(x) = dv = v0 + va*sin(theta) + vb*cos(theta)
        //   s  = [v0 va vb f0 fa fb w theta]
        // H = d/ds * h(s) (Jacobian of measurement function w.r.t. the state).
        // Unlike the old amplitude/phase form, (dh/dva, dh/dvb) = (sin(theta),
        // cos(theta)) can never both be ~0 at once (sin^2+cos^2=1), so there
        // is no coordinate singularity here.
        this->H.setZero();

        this->H( SM::DV, S::V0 )    = 1;
        this->H( SM::DV, S::VA )    = std::sin(x.theta());
        this->H( SM::DV, S::VB )    = std::cos(x.theta());
        this->H( SM::DV, S::F0 )    = 0;
        this->H( SM::DV, S::FA )    = 0;
        this->H( SM::DV, S::FB )    = 0;
        this->H( SM::DV, S::W  )    = 0;
        this->H( SM::DV, S::THETA ) = x.va() * std::cos(x.theta()) - x.vb() * std::sin(x.theta());

    }

};

} // namespace Step
} // namespace KalmanExamples

#endif
