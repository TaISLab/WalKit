#ifndef KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <cmath>

namespace KalmanExamples
{
namespace Step
{

/**
 * @brief System state vector-type (two sines at a common frequency, each
 *        tracked in Cartesian in-phase/quadrature form instead of
 *        amplitude+phase)
 *
 * System is characterized by these two:
 *            dv_k = v0_k + va_k*sin(theta_k) + vb_k*cos(theta_k)
 *            df_k = f0_k + fa_k*sin(theta_k) + fb_k*cos(theta_k)
 *
 * (va,vb) / (fa,fb) are the Cartesian in-phase/quadrature components of
 * each oscillation; amplitude and phase, if ever needed, are derived from
 * them (amplitude = sqrt(a^2+b^2), phase = atan2(b,a)) rather than
 * estimated directly. A previous version of this model carried amplitude
 * (v1/f1) and phase (vp) as states with dv = v0 + v1*sin(vp): that is a
 * *polar* parameterization of the same sinusoid, and polar coordinates
 * have a singularity at the origin -- here, whenever sin(vp) is small the
 * measurement becomes insensitive to v1 (its Jacobian is exactly sin(vp)),
 * so the filter can "explain" a small measurement with an arbitrarily
 * large v1 paired with a near-zero sin(vp) instead of the correct small
 * v1. That produced amplitude estimates that settled on stable but
 * physically nonsensical values (two orders of magnitude above a
 * plausible gait speed) on real data. The Cartesian form removes that singularity: the
 * measurement Jacobian for (va,vb) is (sin(theta), cos(theta)), and
 * sin^2+cos^2=1 always, so at least one of the two is never small.
 *
 * A single shared frequency w and absolute phase theta still drive both
 * oscillations (theta_k+1 = theta_k + w_k*u_k, same as the old vp); the
 * old delay state `d` between the force and speed oscillations is gone,
 * since it is now implicit in how (fa,fb) relate to (va,vb) at the same
 * theta -- no separate parameter is needed to represent it.
 *
 *                     measurements = dv_k (or) df_k = h(k)
 *                           state  = x_k  = [v0_k va_k vb_k f0_k fa_k fb_k w_k theta_k]
 *                         control  = u_k  = t_k+1 - t_k
 *
 *           x_k   = f(x_k-1,u_k-1) = [v0_k va_k vb_k f0_k fa_k fb_k w_k theta_k                ]
 *           x_k+1 =   f(x_k,u_k)   = [v0_k va_k vb_k f0_k fa_k fb_k w_k (theta_k + w_k*u_k)    ]
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 8>
{
public:
    KALMAN_VECTOR(State, T, 8)

    //! v0_k speed diff constant
    static constexpr size_t V0 = 0;
    //! va_k speed diff, in-phase component
    static constexpr size_t VA = 1;
    //! vb_k speed diff, quadrature component
    static constexpr size_t VB = 2;
    //! f0_k force diff constant
    static constexpr size_t F0 = 3;
    //! fa_k force diff, in-phase component
    static constexpr size_t FA = 4;
    //! fb_k force diff, quadrature component
    static constexpr size_t FB = 5;
    //! w_k angular frequency
    static constexpr size_t W = 6;
    //! theta_k absolute phase
    static constexpr size_t THETA = 7;


    T v0()       const { return (*this)[ V0 ]; }
    T va()       const { return (*this)[ VA ]; }
    T vb()       const { return (*this)[ VB ]; }
    T f0()       const { return (*this)[ F0 ]; }
    T fa()       const { return (*this)[ FA ]; }
    T fb()       const { return (*this)[ FB ]; }
    T w()        const { return (*this)[ W  ]; }
    T theta()    const { return (*this)[ THETA ]; }
    T& v0()            { return (*this)[ V0 ]; }
    T& va()             { return (*this)[ VA ]; }
    T& vb()             { return (*this)[ VB ]; }
    T& f0()            { return (*this)[ F0 ]; }
    T& fa()             { return (*this)[ FA ]; }
    T& fb()             { return (*this)[ FB ]; }
    T& w()             { return (*this)[ W  ]; }
    T& theta()          { return (*this)[ THETA ]; }

};

/**
 * @brief System control-input vector-type
 *
 * This is the system control-input defined by time increment.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 1>
{
public:
    KALMAN_VECTOR(Control, T, 1)

    //! time increment
    static constexpr size_t DT = 0;

    T  dt()  const { return (*this)[ DT ]; }
    T& dt() { return (*this)[ DT ]; }
};

/**
 * @brief System model
 *
 * This is the system model defining how our state changes  with
 * control input, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::Step::State<T> S;

    //! Control type shortcut definition
    typedef KalmanExamples::Step::Control<T> C;

    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x Current system state
     * @param [in] u Control input
     * @returns The (predicted) system state given control input and states
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_new_;

        // most of state vars do not change ...
        x_new_ = x;

        // Only absolute phase changes in new state and non-lineally
        auto angle = x.theta() + ( x.w() * u.dt() );
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x_new_.theta() = angle;

        // Return transitioned state vector
        return x_new_;
    }

    // just to save obtaining it several times ...
    static constexpr T dosPi = 2.0 * M_PI;


protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u )
    {
        this->F.setZero();
        // f(v0, va, vb, f0, fa, fb, w, theta, u) = [ v0, va, vb, f0, fa, fb, w, (theta + w*u) ]
        // Every state carries over unchanged except theta, which gains a
        // dependency on w (through u.dt()); everything else is the identity.

        this->F( S::V0,    S::V0    ) = 1;
        this->F( S::VA,    S::VA    ) = 1;
        this->F( S::VB,    S::VB    ) = 1;
        this->F( S::F0,    S::F0    ) = 1;
        this->F( S::FA,    S::FA    ) = 1;
        this->F( S::FB,    S::FB    ) = 1;
        this->F( S::W,     S::W     ) = 1;
        this->F( S::THETA, S::W     ) = u.dt();
        this->F( S::THETA, S::THETA ) = 1;

        // W = df/dw (Jacobian of state transition w.r.t. the noise)

        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as
        //       well as the velocity (i.e. the distance we move)
    }



};

} // namespace Step
} // namespace KalmanExamples

#endif
