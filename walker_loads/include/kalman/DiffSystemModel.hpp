#ifndef KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_LEG_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <cmath>

namespace KalmanExamples
{
namespace Step
{

/**
 * @brief System state vector-type (two sines with unknown amp, phase and delay and common freq,  basically)
 *
 * System is characterized by these two:
 *            dv_k = v0_k + v1_k * sin(w_k*t_k + phi_k)       = v0_k + v1_k * sin(vp_k)
 *            df_k = f0_k + f1_k * sin(w_k*t_k + phi_k + d_k) = f0_k + f1_k * sin(vp_k + d_k)
 * 
 *                     measurements = dv_k (or) df_k = h(k) 
 *                           state  = x_k  = [v0_k v1_k f0_k f1_k w_k d_k  vp_k]
 *                         control  = u_k  = t_k+1 - t_k
 * 
 *           x_k   = f(x_k-1,u_k-1) = [v0_k v1_k f0_k f1_k w_k d_k  vp_k            ] 
 *           x_k+1 =   f(x_k,u_k)   = [v0_k v1_k f0_k f1_k w_k d_k (vp_k + w_k*u_k) ]
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 7>
{
public:
    KALMAN_VECTOR(State, T, 7)
    
    //! v0_k speed diff constant
    static constexpr size_t V0 = 0;
    //! v1_k speed diff amplitude
    static constexpr size_t V1 = 1;
    //! f0_k force diff constant
    static constexpr size_t F0 = 2;
    //! f1_k force diff amplitude
    static constexpr size_t F1 = 3;
    //! w_k angular frequency
    static constexpr size_t W = 4;
    //! d_k force-speed delay
    static constexpr size_t D = 5;
    //! vp_k speed diff absolute phase
    static constexpr size_t VP = 6;


    T v0()       const { return (*this)[ V0 ]; }
    T v1()       const { return (*this)[ V1 ]; }
    T f0()       const { return (*this)[ F0 ]; }
    T f1()       const { return (*this)[ F1 ]; }
    T w()        const { return (*this)[ W  ]; }
    T d()        const { return (*this)[ D  ]; }
    T vp()       const { return (*this)[ VP ]; }    
    T& v0()            { return (*this)[ V0 ]; }
    T& v1()            { return (*this)[ V1 ]; }
    T& f0()            { return (*this)[ F0 ]; }
    T& f1()            { return (*this)[ F1 ]; }
    T& w()             { return (*this)[ W  ]; }
    T& d()             { return (*this)[ D  ]; }
    T& vp()            { return (*this)[ VP ]; }

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
        auto angle = x.vp() + ( x.w() * u.dt() );
        angle = fmod(angle, dosPi);
        if ( angle < 0)
            angle += dosPi;
        x_new_.vp() = angle;

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
        // f(v0, v1, f0, f1, w, d, vp, u) =   [       v0,        v1         f0         f1         w         d, (vp + w*u)]
        // f(v0, v1, f0, f1, w, d, vp, u) =   [     f_v0,      f_v1,      f_f0,      f_f1,      f_w,      f_d,      f_vp]
        // F(v0, v1, f0, f1, w, d, vp, u) = [ [df_v0/dv0, df_v0/dv1, df_v0/df0, df_v0/df1, df_v0/dw, df_v0/dd, df_v0/dvp ],
        //                                    [df_v1/dv0, df_v1/dv1, df_v1/df0, df_v1/df1, df_v1/dw, df_v1/dd, df_v1/dvp ],
        //                                    [df_f0/dv0, df_f0/dv1, df_f0/df0, df_f0/df1, df_f0/dw, df_f0/dd, df_f0/dvp ],
        //                                    [df_f1/dv0, df_f1/dv1, df_f1/df0, df_f1/df1, df_f1/dw, df_f1/dd, df_f1/dvp ],
        //                                    [df_w /dv0, df_w /dv1, df_w /df0, df_w /df1, df_w /dw, df_w /dd, df_w /dvp ],
        //                                    [df_d /dv0, df_d /dv1, df_d /df0, df_d /df1, df_d /dw, df_d /dd, df_d /dvp ],
        //                                    [df_vp/dv0, df_vp/dv1, df_vp/df0, df_vp/df1, df_vp/dw, df_vp/dd, df_vp/dvp ] ]

        this->F( S::V0, S::V0 ) = 1;
        //this->F( S::V0, S::V1 ) = 0;
        //this->F( S::V0, S::F0 ) = 0;
        //this->F( S::V0, S::F1 ) = 0;
        //this->F( S::V0, S::W  ) = 0;
        //this->F( S::V0, S::D  ) = 0;
        //this->F( S::V0, S::VP ) = 0;

        //this->F( S::V1, S::V0 ) = 0;
        this->F( S::V1, S::V1 ) = 1;
        //this->F( S::V1, S::F0 ) = 0;
        //this->F( S::V1, S::F1 ) = 0;
        //this->F( S::V1, S::W  ) = 0;
        //this->F( S::V1, S::D  ) = 0;
        //this->F( S::V1, S::VP ) = 0;

        //this->F( S::F0, S::V0 ) = 0;
        //this->F( S::F0, S::V1 ) = 0;
        this->F( S::F0, S::F0 ) = 1;
        //this->F( S::F0, S::F1 ) = 0;
        //this->F( S::F0, S::W  ) = 0;
        //this->F( S::F0, S::D  ) = 0;
        //this->F( S::F0, S::VP ) = 0;

        //this->F( S::F1, S::V0 ) = 0;
        //this->F( S::F1, S::V1 ) = 0;
        //this->F( S::F1, S::F0 ) = 0;
        this->F( S::F1, S::F1 ) = 1;
        //this->F( S::F1, S::W  ) = 0;
        //this->F( S::F1, S::D  ) = 0;
        //this->F( S::F1, S::VP ) = 0;

        //this->F( S::W, S::V0 ) = 0;
        //this->F( S::W, S::V1 ) = 0;
        //this->F( S::W, S::F0 ) = 0;
        //this->F( S::W, S::F1 ) = 0;
        this->F( S::W, S::W  ) = 1;
        //this->F( S::W, S::D  ) = 0;
        //this->F( S::W, S::VP ) = 0;

        //this->F( S::D, S::V0 ) = 0;
        //this->F( S::D, S::V1 ) = 0;
        //this->F( S::D, S::F0 ) = 0;
        //this->F( S::D, S::F1 ) = 0;
        //this->F( S::D, S::W  ) = 0;
        this->F( S::D, S::D  ) = 1;
        //this->F( S::D, S::VP ) = 0;

        //this->F( S::VP, S::V0 ) = 0;
        //this->F( S::VP, S::V1 ) = 0;
        //this->F( S::VP, S::F0 ) = 0;
        //this->F( S::VP, S::F1 ) = 0;
        this->F( S::VP, S::W  ) = u.dt();
        //this->F( S::VP, S::D  ) = 0;
        this->F( S::VP, S::VP ) = 1;

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
