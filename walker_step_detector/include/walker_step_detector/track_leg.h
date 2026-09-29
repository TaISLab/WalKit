#ifndef TRACKLEG_HH
#define TRACKLEG_HH

#include<set>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"

#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/UnscentedKalmanFilter.hpp"

#include "kalman/SystemModelLeg.hpp"
#include "kalman/PositionMeasurementModelLeg.hpp"
#include "walker_step_detector/compare_steps.h"


// Some type shortcuts
typedef float T;
typedef Leg::State<T> State;
typedef Leg::Control<T> Control;
typedef Leg::SystemModel<T> SystemModel;

typedef Leg::PositionMeasurement<T> PositionMeasurement;
typedef Leg::PositionMeasurementModel<T> PositionModel;

    class TrackLeg{   

        public:            

            TrackLeg();
            
            ~TrackLeg();

            void init(rclcpp::Node *node_, std::string name, double d0, double a0, double f0, double p0,
                      double max_association_dist = 0.5, unsigned int max_consecutive_misses = 15);

            void add( walker_msgs::msg::StepStamped step);

            walker_msgs::msg::StepStamped get_step();

            // Referencia de asociacion/gating: la ULTIMA medida real
            // aceptada por el gate (posicion cruda, sin pasar por el modelo
            // periodico), no la prediccion del EKF (get_step()/curr_step).
            // Motivo (ver track_leg.cpp::init, comentario sobre Q/R e
            // inicializacion): con a0/f0/p0~0 el termino periodico es casi
            // inobservable y su prediccion puede derivar; el 64.3% de los
            // swaps de rf medidos en 47 bags estaban precedidos de esa
            // deriva de referencia (scripts/investigate_ref_drift.py). Usar
            // la ultima medida real como referencia para LegsTracker (coste
            // de emparejamiento) y para el propio gate de predict_step()
            // evita heredar esa deriva sin tocar el filtro en si.
            //
            // Se probo una extension con extrapolacion por velocidad
            // (assoc_ref(ti) = last_accepted_ + velocidad_entre_aceptadas*dt)
            // y empeoro con claridad (47 bags: rf match_rate 94.47/94.87% ->
            // 89.4/88.9%, peor que sin ningun fix en absoluto) -- diferenciar
            // posiciones consecutivas con ruido de medida amplifica ese
            // ruido, y extrapolar con una velocidad ya ruidosa reintroduce
            // mas error del que evita. Revertido; ver track_leg.cpp::predict_step.
            walker_msgs::msg::StepStamped assoc_ref() const {
                return has_accepted_ ? last_accepted_ : curr_step;
            }

            // true una vez se ha aceptado al menos una medida real (ver
            // has_measurement_): antes de eso get_step()/curr_step es un
            // StepStamped por defecto en el origen, sin significado --
            // LegsTracker lo usa para saber si ya puede comparar una
            // deteccion nueva contra la ultima posicion conocida de esta
            // pierna, o si todavia tiene que recurrir a una heuristica de
            // arranque.
            bool has_estimate() const { return has_measurement_; }

            // Cuantas medidas reales (aceptadas por el gate) ha visto esta
            // pista en total, e y media de esas medidas -- NO la posicion
            // actual (que oscila con la zancada), sino un promedio estable
            // pensado para decidir, una vez con datos de sobra, si esta
            // pista corresponde a la salida "izquierda" o "derecha" (ver
            // LegsTracker: decidirlo con el signo de y de un unico frame es
            // ruidoso, un cruce justo al arrancar deja la etiqueta al reves
            // el resto de la sesion).
            unsigned int accepted_count() const { return y_count_; }
            double avg_y() const { return y_count_ ? y_sum_ / y_count_ : 0.0; }

            // Velocidad cruda entre las dos ultimas detecciones anadidas
            // (add()), independiente del EKF/predict_step -- funciona igual
            // con kalman_enabled=false. Pensada para desempatar asociaciones
            // ambiguas por posicion usando la alternancia de marcha: en cada
            // instante una pierna va mas rapido (swing) y la otra mas
            // despacio (stance); ver LegsTracker::add_detections.
            geometry_msgs::msg::Point raw_speed() const { return raw_speed_; }

            int size();

            walker_msgs::msg::StepStamped predict_step(double t);

            geometry_msgs::msg::Point get_speed(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step);
            
            geometry_msgs::msg::Point get_dist(walker_msgs::msg::StepStamped step, walker_msgs::msg::StepStamped prev_step);
            
            void enable_log();

            walker_msgs::msg::StepStamped last_data();
        private:
            // Cuenta un ciclo de predict_step() sin medida aceptada; si se
            // alcanza max_consecutive_misses_, suelta el bloqueo
            // (has_measurement_=false) para que LegsTracker pueda volver a
            // reengancharla con cualquier deteccion disponible.
            void register_miss();

            // Calentamiento de fase/amplitud (propuesta "A", ver
            // detect_steps_algorithm.md): una vez el buffer warmup_* tiene
            // WARMUP_WINDOW medidas reales aceptadas, ajusta a/p por eje
            // por minimos cuadrados (frecuencia fija en WARMUP_FREQ_HZ, la
            // mediana medida sobre mocap real en 47 bags con
            // scripts/estimate_gait_params.py) y reinyecta ese estado en el
            // EKF -- en vez de dejar a0/f0/p0~0 para siempre (lo que hacia
            // el termino periodico casi inobservable, ver el comentario de
            // Q/R en track_leg.cpp::init). d_x/d_y NO se tocan: se dejan
            // como los tiene el EKF ya (bien seguidos por el gain por
            // defecto), solo se resuelve el residuo detrendado.
            void warmup_reseed(double ti);


            std::ofstream myfile;

            // vector where we store laser detections that could be an step detection
            std::vector<walker_msgs::msg::StepStamped> step_list;

            walker_msgs::msg::StepStamped curr_step;

            // last update time
            double t;

            rclcpp::Node *node;

            bool is_init;
            bool is_debug;

            // Gating de asociacion de datos (predict_step): distancia maxima
            // (m) entre la posicion actual seguida y la deteccion candidata
            // para aceptarla como medida del EKF. Evita engancharse a un
            // clister espurio (la otra pierna, ruido, otra persona) cuando
            // la deteccion real de esta pierna falta en el frame.
            double max_association_dist_;
            // false hasta la primera medida aceptada: antes de eso curr_step
            // es un StepStamped por defecto (0,0,0) sin significado, así que
            // no hay nada sensato contra lo que aplicar el gate todavia.
            bool has_measurement_;

            // Recuperacion de pista perdida: si pasan max_consecutive_misses_
            // predict_step() seguidos sin aceptar ninguna medida (por gate o
            // por no haber detecciones), esta pista suelta su bloqueo
            // (has_measurement_ vuelve a false) para que LegsTracker vuelva a
            // dejarla competir por cualquier deteccion disponible en vez de
            // seguir comparando contra una referencia que ya no significa
            // nada. Sin esto, una pista que se desvia una vez no vuelve a
            // engancharse jamas (visto en pruebas reales: swap_rate no
            // mejoraba y una pista se quedaba permanentemente a 0% de acierto).
            unsigned int consecutive_misses_;
            unsigned int max_consecutive_misses_;

            // Acumulado de y de las medidas aceptadas, para avg_y()/accepted_count().
            // Se limita a MAX_Y_SAMPLES para que sea un promedio "reciente"
            // razonable y no crezca sin limite en sesiones largas.
            double y_sum_;
            unsigned int y_count_;
            static const unsigned int MAX_Y_SAMPLES = 30;

            // Para raw_speed(): ultima deteccion anadida y si ya hay alguna
            // previa contra la que calcular una velocidad.
            walker_msgs::msg::StepStamped last_added_;
            bool has_last_added_;
            geometry_msgs::msg::Point raw_speed_;

            // Para assoc_ref(): ultima medida real ACEPTADA por el gate
            // (fijada en predict_step() justo cuando accept==true, con la
            // medida cruda, no con el estado del EKF tras el update) --
            // distinto de last_added_/raw_speed_ arriba, que se actualizan
            // en add() con CUALQUIER deteccion asignada a esta pista ese
            // ciclo, antes de que el propio gate de predict_step() decida
            // si es de fiar. Usar last_added_ aqui seria circular: para el
            // momento en que predict_step() la necesita como referencia,
            // last_added_ ya es la candidata de ESTE ciclo (la que se esta
            // evaluando), no la de la medida aceptada anterior.
            walker_msgs::msg::StepStamped last_accepted_;
            bool has_accepted_;

            // Buffer de medidas ACEPTADAS recientes para warmup_reseed():
            // ti (mismo reloj que u.dt(), no el header.stamp de la medida
            // -- ver el comentario del intento de extrapolacion por
            // velocidad descartado, mismo motivo) + posicion cruda. Se
            // vacia cada vez que la pista pierde el bloqueo (register_miss)
            // para que el ajuste se repita al reenganchar, no solo una vez.
            static const unsigned int WARMUP_WINDOW = 12;
            // Mediana de frecuencia de zancada real medida sobre 47 bags de
            // mocap (scripts/estimate_gait_params.py) -- se probo como a0/f0
            // fijo (sin fase estimada) y empeoro (ver detect_steps_algorithm.md,
            // punto 5); aqui solo se usa para resolver amplitud/fase por
            // minimos cuadrados, no como parametro fijo del modelo.
            //
            // Se probo tambien a ESTIMAR la frecuencia (busqueda en rejilla
            // 0.2-1.2Hz sobre el mismo ajuste) en vez de fijarla aqui, y
            // empeoro con claridad (47 bags: rf match_rate 95.94/95.72% ->
            // 95.61/92.43%, este ultimo peor que sin ningun fix en absoluto
            // -- ver detect_steps_algorithm.md, punto 9). Con solo
            // WARMUP_WINDOW=12 medidas (~1-2 ciclos), dejar la frecuencia
            // libre sobreajusta al ruido de esa ventana concreta; la
            // mediana fija generaliza mejor. Revertido.
            static constexpr double WARMUP_FREQ_HZ = 0.6;
            std::vector<double> warmup_ti_;
            std::vector<double> warmup_x_;
            std::vector<double> warmup_y_;
            // true una vez reinyectado el ajuste para el enganche actual;
            // se vuelve a poner a false en register_miss() al soltar el
            // bloqueo, para recalcular la fase la proxima vez que reenganche.
            bool phase_warmed_;

            std::string name;

            // Extended Kalman Filter
            Kalman::ExtendedKalmanFilter<State> ekf;

            // System model
            SystemModel sys;
    
            // Measurement model
            PositionModel pm;

    };




#endif
