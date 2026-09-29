#include <walker_step_detector/legs_tracker.h>
#include <cmath>
#include <limits>
#include <vector>

    LegsTracker::LegsTracker(){
        is_init = false;
        is_debug = false;
        status_ = false;
        relabel_checked_ = false;
        swap_output_ = false;
    }

    void LegsTracker::init(rclcpp::Node *node_,double d0, double a0, double f0, double p0,
                            double max_association_dist, unsigned int max_consecutive_misses){
        if (!is_init){
            is_init = true;
            node = node_;
            l_tracker.init(node_, "left", d0, a0, f0, p0, max_association_dist, max_consecutive_misses);
            r_tracker.init(node_, "right",d0, a0, f0, p0, max_association_dist, max_consecutive_misses);
        }
    }

    void LegsTracker::enable_log(){
        is_debug = true;
        l_tracker.enable_log();
        r_tracker.enable_log();
    }


    LegsTracker::~LegsTracker(){        
    }

    void LegsTracker::add_detections( std::list<walker_msgs::msg::StepStamped> detect_steps){

        walker_msgs::msg::StepStamped first_step, last_step;

        unsigned int n_points;

        if (is_debug){
            RCLCPP_DEBUG (node->get_logger(), "Adding (%ld) detections", detect_steps.size());
        }

        if (!is_init || detect_steps.empty()){
            return;
        }

        maybe_relabel();

        n_points = detect_steps.size();

        // Con memoria: en cuanto las dos piernas tienen ya una estimacion
        // propia (has_estimate()), las detecciones se asignan comparando
        // contra la ULTIMA posicion conocida de cada pista (get_step()) --
        // no por el signo/orden de "y" en el frame actual. Eso era lo que
        // rompia antes: si las piernas se cruzan en y durante la zancada
        // (habitual con un andador), la asignacion puramente espacial
        // intercambiaba las identidades izquierda/derecha en cada cruce.
        if (l_tracker.has_estimate() && r_tracker.has_estimate()){
            // assoc_ref() (ultima medida real aceptada), no get_step() (la
            // prediccion del modelo periodico): ver el comentario en
            // track_leg.h -- la prediccion puede derivar cuando a0/f0/p0
            // arrancan casi a cero, y esa deriva precedia al 64.3% de los
            // swaps medidos (scripts/investigate_ref_drift.py).
            walker_msgs::msg::StepStamped l_ref = l_tracker.assoc_ref();
            walker_msgs::msg::StepStamped r_ref = r_tracker.assoc_ref();

            std::vector<walker_msgs::msg::StepStamped> dets(detect_steps.begin(), detect_steps.end());

            if (dets.size() == 1){
                double dl = CompareSteps::dist(dets[0], l_ref);
                double dr = CompareSteps::dist(dets[0], r_ref);
                if (dl<=dr) l_tracker.add(dets[0]); else r_tracker.add(dets[0]);
                return;
            }

            // >=2 detecciones: escoge, de TODAS las parejas de detecciones
            // distintas, la que mejor casa con (izquierda, derecha) A LA
            // VEZ (coste = distancia de la una a l_ref + distancia de la
            // otra a r_ref). Asignar cada deteccion por separado a "la mas
            // cercana" deja que las dos pistas compitan por la MISMA
            // deteccion si una de ellas ha derivado un poco -- la otra se
            // queda sin nada, y sin nada no puede corregirse nunca (una
            // pista sin medidas no tiene forma de volver a engancharse).
            // Forzando que cada pierna se quede con una deteccion distinta,
            // la pista que ha derivado sigue recibiendo su mejor candidata
            // disponible cada frame y puede recuperarse.
            size_t best_i = 0, best_j = 1;
            double best_cost = std::numeric_limits<double>::max();
            for (size_t i = 0; i < dets.size(); ++i){
                for (size_t j = 0; j < dets.size(); ++j){
                    if (i == j) continue;
                    double cost = CompareSteps::dist(dets[i], l_ref) + CompareSteps::dist(dets[j], r_ref);
                    if (cost < best_cost){
                        best_cost = cost;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
            // coste de la pareja exactamente contraria a la elegida (izq<->der)
            double alt_cost = CompareSteps::dist(dets[best_j], l_ref) + CompareSteps::dist(dets[best_i], r_ref);

            // Alternancia de marcha como desempate: si la mejor pareja por
            // posicion y la contraria (izquierda<->derecha intercambiadas)
            // tienen un coste muy parecido, la posicion sola no decide con
            // confianza -- es justo lo que pasa cuando las dos piernas se
            // cruzan o estan muy juntas. En ese caso concreto (solo en ese)
            // se usa la velocidad: en cada instante de la marcha una pierna
            // va mas rapido (swing) y la otra mas despacio (stance), asi que
            // se prefiere la asignacion que mantenga el ORDEN de velocidades
            // de la pierna que ya iba mas rapido, en vez de invertirlo de
            // golpe (ver TrackLeg::raw_speed()).
            if (dets.size() >= 2 && best_i != best_j && (alt_cost - best_cost) < AMBIGUITY_MARGIN_M){
                geometry_msgs::msg::Point l_prev_v = l_tracker.raw_speed();
                geometry_msgs::msg::Point r_prev_v = r_tracker.raw_speed();
                double l_prev_speed = std::hypot(l_prev_v.x, l_prev_v.y);
                double r_prev_speed = std::hypot(r_prev_v.x, r_prev_v.y);

                // distancia recorrida desde la ultima posicion conocida, como
                // proxy de velocidad (mismo dt para las dos opciones, asi que
                // sirve igual para compararlas sin necesitar el dt real aqui)
                double d_normal_l = CompareSteps::dist(dets[best_i], l_ref);
                double d_normal_r = CompareSteps::dist(dets[best_j], r_ref);
                double d_alt_l = CompareSteps::dist(dets[best_j], l_ref);
                double d_alt_r = CompareSteps::dist(dets[best_i], r_ref);

                bool l_was_faster = l_prev_speed >= r_prev_speed;
                bool normal_keeps_order = l_was_faster ? (d_normal_l >= d_normal_r) : (d_normal_l <= d_normal_r);
                bool alt_keeps_order    = l_was_faster ? (d_alt_l    >= d_alt_r)    : (d_alt_l    <= d_alt_r);

                if (alt_keeps_order && !normal_keeps_order){
                    std::swap(best_i, best_j);
                    if (is_debug){
                        RCLCPP_DEBUG(node->get_logger(),
                            "Asociacion ambigua por posicion (coste %.3f vs %.3f); desempatada por alternancia de marcha",
                            best_cost, alt_cost);
                    }
                }
                // NOTA sobre reforzar esto con las manetas (walker_msgs/ForceStamped
                // /left_handle, /right_handle): se investigo (scripts/validate_speed_force_link.py)
                // si la asimetria de fuerza en las manetas correlaciona con
                // que pierna esta en swing, usando velocidad REAL de tobillo
                // (mocap, no el detector) contra fuerza real de maneta en 37
                // bags. Resultado: correlacion practicamente nula y sin signo
                // consistente (media~0, positiva en ~45-54% de los bags segun
                // el desfase), y el lag optimo por bag no converge a ningun
                // valor estable (std=0.90s sobre un rango de busqueda de 3s) --
                // la firma de estar ajustando ruido, no una relacion fisica
                // real. Probablemente porque las manetas muestrean a ~2.4Hz,
                // grueso frente al ciclo de zancada (~0.7-1Hz). NO se ha
                // integrado esa senal aqui por esto: anadiria ruido disfrazado
                // de refuerzo. Si en el futuro se dispone de sensores de
                // maneta mas rapidos (>10Hz) valdria la pena repetir esa
                // validacion antes de intentar integrarla como un termino mas
                // en este desempate.
            }

            l_tracker.add(dets[best_i]);
            r_tracker.add(dets[best_j]);

            // El resto (si hay mas de 2 detecciones en el frame) se anaden
            // como medidas extra a la pista mas cercana -- se deja que el
            // EKF (y el gate de TrackLeg::predict_step) decida si las usa.
            for (size_t k = 0; k < dets.size(); ++k){
                if (k == best_i || k == best_j) continue;
                double dl = CompareSteps::dist(dets[k], l_ref);
                double dr = CompareSteps::dist(dets[k], r_ref);
                if (dl<=dr) l_tracker.add(dets[k]); else r_tracker.add(dets[k]);
            }
            return;
        }

        // Recuperacion: exactamente una pierna tiene estimacion fiable --
        // la otra acaba de soltar su bloqueo (TrackLeg::register_miss) o
        // nunca llego a tenerlo, y esta intentando reengancharse. La pierna
        // establecida reclama su vecino mas cercano primero (sin que la que
        // se esta reenganchando le pueda "quitar" su deteccion real); el
        // resto se reparte con la heuristica espacial original, que es la
        // unica pista disponible para una pierna que no tiene referencia
        // propia todavia.
        if (l_tracker.has_estimate() != r_tracker.has_estimate()){
            bool l_est = l_tracker.has_estimate();
            TrackLeg &established = l_est ? l_tracker : r_tracker;
            walker_msgs::msg::StepStamped ref = established.assoc_ref();

            std::vector<walker_msgs::msg::StepStamped> dets(detect_steps.begin(), detect_steps.end());
            size_t best = 0;
            double best_d = CompareSteps::dist(dets[0], ref);
            for (size_t i = 1; i < dets.size(); ++i){
                double d = CompareSteps::dist(dets[i], ref);
                if (d < best_d){
                    best_d = d;
                    best = i;
                }
            }
            established.add(dets[best]);

            for (size_t i = 0; i < dets.size(); ++i){
                if (i == best) continue;
                // left should have y>0
                if (dets[i].position.point.y>0) {
                    l_tracker.add(dets[i]);
                } else{
                    r_tracker.add(dets[i]);
                }
            }
            return;
        }

        // Arranque: ninguna de las dos piernas tiene estimacion propia
        // todavia, asi que no hay nada fiable contra lo que comparar. Se
        // usa la heuristica espacial original solo para dar el primer
        // empujon (asume el andador mirando hacia +x, izquierda con y>0);
        // en cuanto ambas pistas tengan una medida aceptada, el bloque de
        // arriba toma el relevo.
        if (n_points==1){
            first_step = detect_steps.front();
            // left should have y>0
            if (first_step.position.point.y>0) {
                l_tracker.add(first_step);
            } else{
                r_tracker.add(first_step);
            }
        } else if (n_points==2) {
            first_step = detect_steps.front();
            last_step = detect_steps.back();

            // left should have bigger y
            if (first_step.position.point.y>last_step.position.point.y) {
                l_tracker.add(first_step);
                r_tracker.add(last_step);
            } else{
                l_tracker.add(last_step);
                r_tracker.add(first_step);
            }
        } else { // 3 points at least ...
            detect_steps.sort(CompareSteps()); // sorted right-left

            // add all to kalman and let it smooth it
            while(detect_steps.size()>0){
                first_step = detect_steps.front();
                detect_steps.pop_front();
                r_tracker.add(first_step);
                if (detect_steps.size()>0){
                    last_step = detect_steps.back();
                    detect_steps.pop_back();
                    l_tracker.add(last_step);
                }
            }
        }
    }

    void LegsTracker::set_status(bool new_status){
        status_ = new_status;
    }

    void LegsTracker::maybe_relabel(){
        if (relabel_checked_){
            return;
        }
        if (l_tracker.accepted_count() < RELABEL_MIN_SAMPLES || r_tracker.accepted_count() < RELABEL_MIN_SAMPLES){
            return;
        }
        // Ya hay suficientes medidas reales acumuladas en las dos pistas:
        // decide, UNA vez, cual de las dos corresponde de verdad a la
        // salida "izquierda" (la de mayor y media), en vez de fiarse del
        // signo de y de la primerisima deteccion (ruidoso: un cruce o una
        // deteccion espuria justo al arrancar deja la etiqueta al reves
        // para toda la sesion -- visto en las metricas reales: alguna
        // variante terminaba con la salida "izquierda" enganchada al
        // tobillo derecho la mayor parte del bag).
        relabel_checked_ = true;
        swap_output_ = (l_tracker.avg_y() < r_tracker.avg_y());
        if (is_debug && swap_output_){
            RCLCPP_DEBUG(node->get_logger(),
                "Re-etiquetando salidas: l_tracker.avg_y=%.3f < r_tracker.avg_y=%.3f",
                l_tracker.avg_y(), r_tracker.avg_y());
        }
    }

    void LegsTracker::get_current_refs(walker_msgs::msg::StepStamped* left_ref, walker_msgs::msg::StepStamped* right_ref){
        TrackLeg &left_output  = swap_output_ ? r_tracker : l_tracker;
        TrackLeg &right_output = swap_output_ ? l_tracker : r_tracker;
        *left_ref = left_output.get_step();
        *right_ref = right_output.get_step();
    }

    void LegsTracker::get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l, double t){

        TrackLeg &left_output  = swap_output_ ? r_tracker : l_tracker;
        TrackLeg &right_output = swap_output_ ? l_tracker : r_tracker;

        if (status_){
            if (is_debug){
                RCLCPP_DEBUG (node->get_logger(), "Prediction requested at time (%3.3f)",t*1e-9);
            }

            *step_r = right_output.predict_step(t);
            *step_l = left_output.predict_step(t);

        } else {
            if (is_debug){
                RCLCPP_DEBUG (node->get_logger(), "Last data requested !");
            }

            *step_r = right_output.last_data();
            *step_l = left_output.last_data();


        }


    }


    unsigned int LegsTracker::data_size(){
        unsigned int stored_steps;
        stored_steps = 0;
        if (is_init){
             stored_steps = r_tracker.size() + l_tracker.size();
             stored_steps = stored_steps >> 2;
        }

        return stored_steps;
    }


