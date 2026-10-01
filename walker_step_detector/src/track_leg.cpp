
#include <walker_step_detector/track_leg.h>
#include <cmath>


    TrackLeg::TrackLeg(){
        is_init=false;
        is_debug=false;
        has_measurement_=false;
        consecutive_misses_=0;
        y_sum_=0.0;
        y_count_=0;
        has_last_added_=false;
        has_accepted_=false;
        phase_warmed_=false;
    }

    void TrackLeg::enable_log(){
        is_debug = true;
        myfile.open (name + "_measurements.csv");
    }

    void TrackLeg::init(rclcpp::Node *node_, std::string name_, double d0, double a0, double f0, double p0,
                         double max_association_dist, unsigned int max_consecutive_misses){

        if (!is_init){
            is_init=true;
            node = node_;
            t = 0;
            name = name_;
            max_association_dist_ = max_association_dist;
            max_consecutive_misses_ = max_consecutive_misses;
            // System initial state
            State x0;

            x0.d_x() = d0;   // meters
            x0.a_x() = a0;   // meters
            x0.f_x() = f0;  // hertzs
            x0.p_x() = p0;  // rads

            x0.d_y() = d0;   // meters
            x0.a_y() = a0;   // meters
            x0.f_y() = f0;  // hertzs
            x0.p_y() = p0;  // rads

            // Init filter with system state
            ekf.init(x0);

            // Q/R (ruido de proceso/medida) probados y descartados: sys/pm
            // nunca llaman a setCovariance(), asi que se quedan en el
            // Identity de StandardBase (varianza=1 en cada componente, muy
            // por encima del ruido real). Se probo a fijar valores
            // fisicamente razonados (offset/amplitud ~2cm, frecuencia
            // ~0.05Hz, fase ~0.1rad, medida ~3cm) y, medido limpiamente en
            // 47 bags (mismo dominio ROS aislado, antes y despues -- ver
            // eval_results/result_2026-09-28_baseline_isolated.json vs
            // result_2026-09-28_qr_tuned_isolated.json), empeoro las tres
            // metricas de rf de forma consistente (match_rate 93.7/93.8% ->
            // 85.2/85.4%). Motivo probable: a0/f0/p0 arrancan practicamente a
            // cero, y el jacobiano de medida respecto a la frecuencia es
            // proporcional a la amplitud (dh/df = 2*pi*a*cos(p)) -- con
            // amplitud ~0 la frecuencia es casi inobservable, asi que en la
            // practica es el termino D (offset) el que hace todo el trabajo
            // de seguimiento, y solo consigue perseguir la medida real
            // porque el Q=Identity por defecto le da una ganancia de Kalman
            // grande; un Q mas pequeño (fisicamente mas correcto) frena esa
            // convergencia sin arreglar la causa real (identificabilidad
            // desde una inicializacion casi nula). Arreglarlo de verdad
            // necesitaria repensar la inicializacion o la parametrizacion
            // del modelo, no solo Q/R.
        }

        /*
                import matplotlib.pyplot as plt  
                import pandas as pd      
                import numpy as np    

                data = pd.read_csv('left.csv', na_values='nan', names=['dt', 'x','y'])       
                data['t'] = np.cumsum(data.dt)

                data = pd.read_csv('right.csv', na_values='nan', names=['dt', 'x','y'])
                data['t'] = np.cumsum(data.dt)

                plt.plot(data.t, data.x); plt.show()
                100*np.sum(np.isfinite(data.x))/len(data.x)


         */
    }

    // TrackLeg::TrackLeg(){
    //     t = 0;
    // }
            
    TrackLeg::~TrackLeg(){
        if (is_debug)
            myfile.close();
    }

    void TrackLeg::add( walker_msgs::msg::StepStamped step){
        if (is_init){
            step_list.push_back(step);
            // acumulado para avg_y()/accepted_count() (ver LegsTracker::maybe_relabel):
            // aqui y no en predict_step() para que funcione tambien con
            // kalman_enabled=false (esa rama nunca llama a predict_step,
            // solo a last_data()).
            if (y_count_ < MAX_Y_SAMPLES){
                y_sum_ += step.position.point.y;
                y_count_++;
            }
            // raw_speed(): igual que avg_y(), calculado aqui (no en
            // predict_step) para que exista tambien con kalman_enabled=false.
            if (has_last_added_){
                raw_speed_ = get_speed(step, last_added_);
            }
            last_added_ = step;
            has_last_added_ = true;
            //RCLCPP_ERROR (node->get_logger(), "(%s) has new step  [%3.3f, %3.3f, (%s)]",name.c_str(), step.position.point.x, step.position.point.y, step.position.header.frame_id.c_str());
        }
    }

    walker_msgs::msg::StepStamped TrackLeg::last_data(){
        walker_msgs::msg::StepStamped ans;
        if (is_init){
            if (step_list.size()){
                ans = step_list.back();
            } else {
                ans.position.header.frame_id = "invalid";
            }
        }

        return ans;        
    }

    walker_msgs::msg::StepStamped TrackLeg::get_step(){
        return curr_step;
    }

    void TrackLeg::register_miss(){
        if (!has_measurement_){
            return; // ya sin bloqueo, nada que soltar
        }
        consecutive_misses_++;
        if (consecutive_misses_ >= max_consecutive_misses_){
            if (is_debug){
                RCLCPP_DEBUG(node->get_logger(),
                    "(%s) sin medida aceptada en %d ciclos seguidos. Soltando bloqueo para reenganchar.",
                    name.c_str(), consecutive_misses_);
            }
            has_measurement_ = false;
            consecutive_misses_ = 0;
            // Al soltar el bloqueo, el proximo enganche es tan "nuevo" como
            // el primero: la fase real de la zancada en ese momento no
            // tiene por que guardar relacion con la de antes de perderla.
            // Vaciar el buffer para que warmup_reseed() se recalcule desde
            // cero en vez de mezclar medidas de dos enganches distintos.
            warmup_ti_.clear();
            warmup_x_.clear();
            warmup_y_.clear();
            phase_warmed_ = false;
        }
    }

    void TrackLeg::warmup_reseed(double ti){
        State x0 = ekf.getState();
        double d_x_cur = x0.d_x();
        double d_y_cur = x0.d_y();
        const double omega = 2.0 * M_PI * WARMUP_FREQ_HZ;

        // Minimos cuadrados de (pos - d_actual) ~= A*sin(w*t_rel) + B*cos(w*t_rel),
        // t_rel relativo a `ti` (la ultima medida del buffer tiene t_rel~0):
        // el p resultante (atan2(B,A)) es entonces la fase absoluta valida
        // EN `ti`, lista para inyectarse tal cual (ver State::f() en
        // SystemModelLeg.hpp: p es fase absoluta, se integra sumando
        // 2*pi*f*dt en cada predict, no se recalcula desde un origen).
        //
        // Se probo tambien a estimar la frecuencia (no solo amplitud/fase)
        // con una busqueda en rejilla sobre este mismo ajuste (0.2-1.2Hz,
        // paso 0.02Hz, residuo total x+y por candidata) y empeoro con
        // claridad (47 bags: rf match_rate 95.94/95.72% -> 95.61/**92.43%**,
        // este ultimo PEOR que el baseline sin ningun fix, 93.81%;
        // eval_results/result_2026-09-29_warmup_freq_grid_isolated.json).
        // Motivo probable: con solo WARMUP_WINDOW=12 medidas (~1-2 ciclos
        // de marcha), dejar la frecuencia libre añade un grado de libertad
        // que sobreajusta al ruido de ESA ventana concreta en vez de
        // converger a la frecuencia real -- la mediana de 47 bags de mocap
        // (0.6Hz, fija) generaliza mejor que una estimacion de una sola
        // ventana corta y ruidosa. Revertido a frecuencia fija.
        double sum_ss=0, sum_sc=0, sum_cc=0;
        double sum_ys_x=0, sum_yc_x=0, sum_ys_y=0, sum_yc_y=0;
        for (size_t i = 0; i < warmup_ti_.size(); ++i){
            double t_rel = (warmup_ti_[i] - ti) * 1e-9;
            double s = std::sin(omega * t_rel);
            double c = std::cos(omega * t_rel);
            double yx = warmup_x_[i] - d_x_cur;
            double yy = warmup_y_[i] - d_y_cur;
            sum_ss += s*s; sum_sc += s*c; sum_cc += c*c;
            sum_ys_x += yx*s; sum_yc_x += yx*c;
            sum_ys_y += yy*s; sum_yc_y += yy*c;
        }
        double det = sum_ss*sum_cc - sum_sc*sum_sc;
        if (std::fabs(det) < 1e-9){
            return; // ventana mal condicionada (poca variacion de fase): no reinyectar
        }

        double Ax = (sum_ys_x*sum_cc - sum_yc_x*sum_sc) / det;
        double Bx = (sum_ss*sum_yc_x - sum_sc*sum_ys_x) / det;
        double Ay = (sum_ys_y*sum_cc - sum_yc_y*sum_sc) / det;
        double By = (sum_ss*sum_yc_y - sum_sc*sum_ys_y) / det;

        x0.a_x() = std::sqrt(Ax*Ax + Bx*Bx);
        x0.p_x() = std::atan2(Bx, Ax);
        x0.f_x() = WARMUP_FREQ_HZ;
        x0.a_y() = std::sqrt(Ay*Ay + By*By);
        x0.p_y() = std::atan2(By, Ay);
        x0.f_y() = WARMUP_FREQ_HZ;
        // d_x/d_y: sin tocar (x0 ya los conserva, get/set no los toca arriba).

        ekf.init(x0); // solo pisa el estado (x); no toca la covarianza (P).

        if (is_debug){
            RCLCPP_DEBUG(node->get_logger(),
                "(%s) warmup_reseed: a=(%.3f,%.3f) p=(%.3f,%.3f) f=%.2fHz",
                name.c_str(), x0.a_x(), x0.a_y(), x0.p_x(), x0.p_y(), WARMUP_FREQ_HZ);
        }
    }

    int TrackLeg::size(){
        return step_list.size();
    }

    walker_msgs::msg::StepStamped TrackLeg::predict_step(double ti){
        walker_msgs::msg::StepStamped pred_step, measure_step;

        if (is_debug)
            RCLCPP_DEBUG(node->get_logger(), "Predicting step position");

        if (is_init){
            // Control input
            Control u;

            // is it a tracked measurement or just a prediction?
            pred_step.tracked = false;
            // how sure are we this is a "leg"
            pred_step.confidence = curr_step.confidence;
            pred_step.position.point.z = curr_step.position.point.z;

            // where?
            pred_step.position.header = curr_step.position.header;

            // t (miembro, "last update time") se inicializa a 0 en init()
            // -- ES un sentinel de "todavia no se ha predicho nunca", NO un
            // instante real. Si se usa tal cual en la primerisima llamada,
            // u.dt() = (ti-0)*1e-9 sale como el tiempo desde el epoch Unix
            // (~1.79e9 s), no como un intervalo real entre frames: ese dt
            // gigante entra en el jacobiano de SystemModelLeg.hpp
            // (F(PX,FX) = 2*pi*dt) y dispara una covarianza P descontrolada
            // desde el primer ciclo -- confirmado en vivo (CA_test09,
            // km_detect_steps con kalman_enabled=true): la pista "right"
            // salta a y=407m en la SEGUNDA medida, con una candidata cruda
            // de entrada perfectamente normal (~1m, comprobado con
            // kalman_enabled=false en el mismo bag). rf no lo mostraba en
            // este bag por pura coincidencia de timing (su primera medida
            // aceptada no caia justo en el ciclo con la P recien inflada),
            // no porque el bug no le afectase -- es el mismo codigo
            // compartido. Fix: en la primerisima llamada, tratar dt como 0
            // en vez de como "tiempo desde el epoch".
            if (t == 0){
                t = ti;
            }

            // Predict state for current time-step using the filters
            u.dt() = (ti-t)*1e-9;

            auto ekf_state = ekf.predict(sys, u);

            // option 1: consider closest detection 
            //           to current step position as
            //           kalman filter measurement

            if (is_debug)
                RCLCPP_DEBUG(node->get_logger(), "%ld measurements available for EFK update", step_list.size());

            if (step_list.size() > 0) {
                // find closest to the association reference -- capturada
                // ANTES de que este ciclo pueda actualizarla (ver
                // assoc_ref()/last_accepted_ en track_leg.h): usar
                // curr_step aqui (prediccion del modelo periodico) fue lo
                // que dejaba esta busqueda seguir una referencia que ya
                // habia derivado.
                //
                // Se probo a extrapolar ref con la velocidad entre las dos
                // ultimas medidas aceptadas (assoc_ref(ti), diferenciando
                // posiciones consecutivas) y empeoro con claridad (47 bags,
                // rf match_rate 94.47/94.87% -> 89.4/88.9%, PEOR que el
                // baseline original 93.68/93.81%): diferenciar posiciones
                // con ruido de medida amplifica ese ruido (mas cuanto mas
                // corto el intervalo entre medidas), y extrapolar con una
                // velocidad ya ruidosa reintroduce mas error del que evita.
                // Revertido a la posicion sola (sin extrapolar).
                walker_msgs::msg::StepStamped ref = assoc_ref();
                double d = 99999;
                double min_dist = 99999;
                for (auto st : step_list) {
                    d = CompareSteps::dist(ref, st);
                    if (d<min_dist){
                        min_dist = d;
                        measure_step = st;
                    }
                }

                // Gating: la deteccion mas cercana solo se acepta como
                // medida si esta a menos de max_association_dist_ de donde
                // seguiamos esta pierna. Sin esto, si la deteccion real de
                // ESTA pierna falta en el frame (oclusion, filtrada por el
                // clasificador...) pero hay otra deteccion cualquiera en la
                // lista (la otra pierna, ruido, otra persona), el tracker
                // se "engancha" a ella igualmente por ser la menos mala de
                // las disponibles, aunque este a metros de distancia.
                // Siempre se acepta la primerisima medida (has_measurement_
                // aun false): antes de esa primera medida curr_step es un
                // StepStamped por defecto en el origen, sin significado, no
                // hay nada sensato contra lo que comparar la distancia.
                bool accept = (!has_measurement_) || (min_dist <= max_association_dist_);

                if (accept) {
                    pred_step.confidence = measure_step.confidence;
                    pred_step.position.header = measure_step.position.header;

                    PositionMeasurement position;
                    position.pos_x() = measure_step.position.point.x;
                    position.pos_y() = measure_step.position.point.y;
                    pred_step.position.point.z = measure_step.position.point.z;

                    // Update EKF using measurement
                    ekf_state = ekf.update(pm, position);
                    pred_step.tracked = true;
                    has_measurement_ = true;
                    consecutive_misses_ = 0;
                    // assoc_ref() para el PROXIMO ciclo: la medida cruda
                    // aceptada ahora, no el estado del EKF tras el update.
                    last_accepted_ = measure_step;
                    has_accepted_ = true;

                    // Calentamiento de fase/amplitud (propuesta "A"): se
                    // acumula esta medida y, en cuanto el buffer se llena
                    // (y no se ha hecho ya para este enganche), se ajusta
                    // a/p por eje y se reinyecta -- ver warmup_reseed().
                    if (!phase_warmed_){
                        warmup_ti_.push_back(ti);
                        warmup_x_.push_back(measure_step.position.point.x);
                        warmup_y_.push_back(measure_step.position.point.y);
                        if (warmup_ti_.size() >= WARMUP_WINDOW){
                            warmup_reseed(ti);
                            phase_warmed_ = true;
                            warmup_ti_.clear();
                            warmup_x_.clear();
                            warmup_y_.clear();
                        }
                    }
                    // u,x,y   // predict + update
                    if (is_debug)
                        myfile   << u.dt()     << "," << position.pos_x() << "," << position.pos_y() << std::endl;
                } else {
                    // u,nan,nan // predict - (rechazada por el gate)
                    register_miss();
                    if (is_debug){
                        myfile   << u.dt()     << "," << "nan" << "," << "nan" << std::endl;
                        RCLCPP_DEBUG(node->get_logger(),
                            "(%s) deteccion mas cercana a %.2f m > max_association_dist (%.2f m). Rechazada, solo prediccion.",
                            name.c_str(), min_dist, max_association_dist_);
                    }
                }
            } else{
                // u,nan,nan // predict -
                register_miss();
                if (is_debug){
                    myfile   << u.dt()     << "," << "nan" << "," << "nan" << std::endl;
                    RCLCPP_DEBUG(node->get_logger(), "No laser measurement available. No EKF update.");
                }
            }



            PositionMeasurement pred_position = pm.h(ekf_state);

            pred_step.position.point.x = pred_position.pos_x();
            pred_step.position.point.y = pred_position.pos_y();
            
            
            // set prediction time
            pred_step.position.header.stamp = rclcpp::Time(ti);

            // get speeds
            pred_step.speed = get_speed(pred_step, curr_step);
            
            // cleaning: my new reference is this one
            curr_step = pred_step;

            // new list, forget previous potential detections ...
            step_list.clear();     

            // store last prediction time    
            t = ti;
        }
        if (is_debug){
            RCLCPP_DEBUG(node->get_logger(), "Pred (%s) step at  [%3.3f, %3.3f, (%s)]",name.c_str(), pred_step.position.point.x, pred_step.position.point.y, pred_step.position.header.frame_id.c_str());
        }
        return pred_step;
    }

    geometry_msgs::msg::Point TrackLeg::get_speed(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step)
    {
        
        double inc_t, st, pst;
        geometry_msgs::msg::Point vel;
        vel.x = vel.y = vel.z = 0;

        // check confidence problems
        // if (step.confidence==0) {
        //         RCLCPP_ERROR(node->get_logger(), "get_speed: step confidence == 0 ");    
        //         return vel;
        // }
        // if (prev_step.confidence==0){
        //         RCLCPP_ERROR(node->get_logger(), "get_speed: prev step confidence == 0 ");    
        //         return vel;
        // }

        // check timestamp problems
        st = step.position.header.stamp.sec + step.position.header.stamp.nanosec*1e-9;
        pst = prev_step.position.header.stamp.sec + prev_step.position.header.stamp.nanosec*1e-9;

        if (st==0) {
                RCLCPP_WARN(node->get_logger(), "get_speed: step timestamp == 0 ");    
                return vel;
        }
        if (pst==0){
                RCLCPP_WARN(node->get_logger(), "get_speed: prev step timestamp == 0 ");    
                return vel;
        }
        if ( st == pst){
                RCLCPP_WARN(node->get_logger(), "get_speed: both timestamps are equal ");    
                return vel;
        }

        // data is sane.
        inc_t = st - pst;

        vel = get_dist( step, prev_step);
        vel.x = vel.x / inc_t;
        vel.y = vel.y / inc_t;
        vel.z = vel.z / inc_t;                        
     
        return vel;
    }    

    geometry_msgs::msg::Point TrackLeg::get_dist(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step){
        
        double inc_x, inc_y, inc_z;
        geometry_msgs::msg::Point ans;
        ans.x = ans.y = ans.z = 0;

        // check confidence problems
        // if (step.confidence==0) {
        //         RCLCPP_ERROR(node->get_logger(), "get_dist: step confidence == 0 ");    
        //         return ans;
        // }
        // if (prev_step.confidence==0){
        //         RCLCPP_ERROR(node->get_logger(), "get_dist: prev step confidence == 0 ");    
        //         return ans;
        // }

        inc_x = step.position.point.x - prev_step.position.point.x;
        inc_y = step.position.point.y - prev_step.position.point.y;
        inc_z = step.position.point.z - prev_step.position.point.z;

        ans.x = inc_x;
        ans.y = inc_y;
        ans.z = inc_z;                        
     
        return ans;
    }    