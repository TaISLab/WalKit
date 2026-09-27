#ifndef KALMAN_EXAMPLES_LEG_FREQUENCYMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_LEG_FREQUENCYMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace KalmanExamples
{
namespace Step
{

/**
 * @brief Measurement of the gait's angular frequency (w), estimated
 * externally from step-alternation timing (see
 * DiffTracker::add_frequency_measurement) rather than inferred purely from
 * curve-fitting the force/speed oscillations.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class FrequencyMeasurement : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(FrequencyMeasurement, T, 1)

    //! current measured angular frequency (rad/s)
    static constexpr size_t DW = 0;
    T  dw() const { return (*this)[ DW ]; }
    T& dw()       { return (*this)[ DW ]; }
};

/**
 * @brief Frequency measurement model
 *
 * h(x) = w: a direct, linear measurement of the state's own w. Neither the
 * force nor speed measurement models have any Jacobian sensitivity to w
 * (see DiffSystemModel.hpp) -- w can only move through weak, indirect
 * coupling via theta, which in practice let it drift toward zero (freezing
 * theta and reintroducing the amplitude/phase degeneracy the Cartesian
 * (va,vb)/(fa,fb) parameterization was meant to fix). This model gives w a
 * real, unambiguous observation channel, fed by a period estimate computed
 * from step-alternation timing (a zero-crossing detector on the L/R speed
 * difference, in PartialLoads::steps_lc) instead of by curve-fitting.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class FrequencyMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, FrequencyMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::Step::State<T> S;

    //! Measurement type shortcut definition
    typedef KalmanExamples::Step::FrequencyMeasurement<T> FQ;

    FQ h(const S& x) const
    {
        FQ measurement;
        measurement.dw() = x.w();
        return measurement;
    }

protected:

    void updateJacobians( const S& x )
    {
        (void)x;
        this->H.setZero();
        this->H( FQ::DW, S::W ) = 1;
    }

};

} // namespace Step
} // namespace KalmanExamples

#endif
