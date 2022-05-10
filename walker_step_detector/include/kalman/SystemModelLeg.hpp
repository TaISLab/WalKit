#ifndef KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_

#include "kalman/LinearizedSystemModel.hpp"
#include <cmath>

namespace Leg
{

/**
 * @brief System state vector-type for a leg in swinging in 1D (a sine with unknown freq, amp and phase, basically)
 *
 * System is characterized by its swing amplitude, angular speed and phase:
 *            z_k = a_k * sin(2*pi* f_k*t_k + phi) = a_k sin(p_k)
 *                     measurement = z_k  = h(k) 
 *                           state = x_k  = [a_k  f_k  p_k] = f(x_k-1,u_k-1)
 *                         control = u_k  = t_k+1 - t_k
 *                    f(x_k,u_k)   = [a_k  f_k  p_k + 2*pi*u_k]
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(State, T, 6)
    
    //! Amplitude
    static constexpr size_t AX = 0;
    //! Frequency
    static constexpr size_t FX = 1;
    //! Absolute phase
    static constexpr size_t PX = 2;

    //! Amplitude
    static constexpr size_t AY = 3;
    //! Frequency
    static constexpr size_t FY = 4;
    //! Absolute phase
    static constexpr size_t PY = 5;


    T a_x()       const { return (*this)[ AX ]; }
    T f_x()       const { return (*this)[ FX ]; }
    T p_x()       const { return (*this)[ PX ]; }
    T a_y()       const { return (*this)[ AY ]; }
    T f_y()       const { return (*this)[ FY ]; }
    T p_y()       const { return (*this)[ PY ]; }
    
    T& a_x()      { return (*this)[ AX ]; }
    T& f_x()      { return (*this)[ FX ]; }
    T& p_x()      { return (*this)[ PX ]; }     
    T& a_y()      { return (*this)[ AY ]; }
    T& f_y()      { return (*this)[ FY ]; }
    T& p_y()      { return (*this)[ PY ]; }    

};

/**
 * @brief System control-input vector-type for a leg in 1D
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
    
    //! time 
    static constexpr size_t DT = 0;
    
    T  dt()  const { return (*this)[ DT ]; }    
    T& dt() { return (*this)[ DT ]; }
};

/**
 * @brief System model for a leg swinging in 1D
 *
 * This is the system model defining how our leg changes state with
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
	typedef Leg::State<T> S;
    
    //! Control type shortcut definition
    typedef Leg::Control<T> C;
    
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
                
        x_new_.a_x() = x.a_x();
        x_new_.f_x() = x.f_x();
        x_new_.a_y() = x.a_y();
        x_new_.f_y() = x.f_y();

        // Only absolute phase changes in new state and non-lineally
        auto angle = x.p_x() + ( dosPi * x.f_x() * u.dt() );
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x_new_.p_x() = angle;

        angle = x.p_y() + ( dosPi * x.f_y() * u.dt() );
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x_new_.p_y() = angle;

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
        
        // f(a_x,f_x,p_x,a_y,f_y,p_y) = [f_a_x, f_f_x, f_p_x, f_a_y, f_f_y, f_p_y]
        // F(a_x,f_x,p_x,a_y,f_y,p_y) = [ [df_a_x/da_x, df_a_x/df_x, df_a_x/dp_x, df_a_x/da_y, df_a_x/df_y, df_a_x/dp_y ], 
        //                                [df_f_x/da_x, df_f_x/df_x, df_f_x/dp_x, df_f_x/da_y, df_f_x/df_y, df_f_x/dp_y ], 
        //                                [df_p_x/da_x, df_p_x/df_x, df_p_x/dp_x, df_p_x/da_y, df_p_x/df_y, df_p_x/dp_y ], 
        //                                [df_a_y/da_x, df_a_y/df_x, df_a_y/dp_x, df_a_y/da_y, df_a_y/df_y, df_a_y/dp_y ], 
        //                                [df_f_y/da_x, df_f_y/df_x, df_f_y/dp_x, df_f_y/da_y, df_f_y/df_y, df_f_y/dp_y ], 
        //                                [df_p_y/da_x, df_p_y/df_x, df_p_y/dp_x, df_p_y/da_y, df_p_y/df_y, df_p_y/dp_y ]  ]

          this->F( S::AX, S::AX ) = 1;
        //this->F( S::AX, S::FX ) = 0;
        //this->F( S::AX, S::PX ) = 0;
        //this->F( S::FX, S::AX ) = 0;
          this->F( S::FX, S::FX ) = 1;
        //this->F( S::FX, S::PX ) = 0;
        //this->F( S::PX, S::AX ) = 0;
          this->F( S::PX, S::FX ) = dosPi * u.dt();
          this->F( S::PX, S::PX ) = 1;

          this->F( S::AY, S::AY ) = 1;
        //this->F( S::AY, S::FY ) = 0;
        //this->F( S::AY, S::PY ) = 0;
        //this->F( S::FY, S::AY ) = 0;
          this->F( S::FY, S::FY ) = 1;
        //this->F( S::FY, S::PY ) = 0;
        //this->F( S::PY, S::AY ) = 0;
          this->F( S::PY, S::FY ) = dosPi * u.dt();
          this->F( S::PY, S::PY ) = 1;

        // W = df/dw (Jacobian of state transition w.r.t. the noise)

        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)

        //If you learn how to get rid of [-Wunused-variable] warning: remove these lines
        double nothing = x.a_x();
        nothing += 42;

    }



};

} // namespace Leg

#endif
