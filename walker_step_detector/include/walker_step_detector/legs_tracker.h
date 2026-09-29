#ifndef LEGSTRACKER_HH
#define LEGSTRACKER_HH


#include <rclcpp/rclcpp.hpp>
// Custom Messages related Headers
#include "walker_msgs/msg/step_stamped.hpp"
#include <walker_step_detector/track_leg.h>


    class LegsTracker{   

        public:
            LegsTracker();
                        
            ~LegsTracker();

            void init(rclcpp::Node *node_,double d0, double a0, double f0, double p0,
                      double max_association_dist = 0.5, unsigned int max_consecutive_misses = 15);
            
            void add_detections( std::list<walker_msgs::msg::StepStamped> detect_steps);

            void get_steps(walker_msgs::msg::StepStamped* step_r, walker_msgs::msg::StepStamped* step_l, double t);

            // Referencia interna ACTUAL de cada pista (get_step(), sin
            // predecir ni anadir nada -- lo que add_detections() usara como
            // l_ref/r_ref en el PROXIMO ciclo), ya mapeada segun
            // swap_output_ para que "left"/"right" aqui signifiquen lo mismo
            // que en /detected_step_*_left|right. Pura instrumentacion para
            // depurar swaps offline (ver scripts/investigate_swaps.py): no
            // cambia nada del comportamiento del tracker.
            void get_current_refs(walker_msgs::msg::StepStamped* left_ref, walker_msgs::msg::StepStamped* right_ref);
            void set_status(bool new_status);
            void enable_log();
            unsigned int data_size();
            bool is_init;
            bool status_;
        private:
            // Comprueba, una vez las dos pistas tienen datos de sobra, si
            // hay que invertir que pista alimenta la salida "izquierda"
            // frente a "derecha" (ver swap_output_).
            void maybe_relabel();

            TrackLeg l_tracker;
            TrackLeg r_tracker;
            rclcpp::Node *node;

            bool is_debug;

            // l_tracker/r_tracker siguen cada una a un candidato concreto de
            // forma consistente desde el primer momento (gracias a la
            // asociacion con memoria de add_detections) -- lo unico que no
            // se sabe de entrada es CUAL de las dos corresponde de verdad a
            // la salida "izquierda". Decidirlo con el signo de y de la
            // primerisima deteccion es ruidoso (un cruce o deteccion
            // espuria justo al arrancar fija la etiqueta al reves para toda
            // la sesion). En vez de eso, get_steps() publica segun
            // swap_output_, que maybe_relabel() fija UNA vez, cuando ambas
            // pistas ya tienen varias medidas reales acumuladas (ver
            // TrackLeg::avg_y()) -- promedio, no una muestra suelta.
            bool relabel_checked_;
            bool swap_output_;
            static const unsigned int RELABEL_MIN_SAMPLES = 5;

            // Margen (m, combinado entre las dos piernas) por debajo del cual
            // la mejor pareja por posicion y la contraria se consideran
            // "empatadas" -- ahi es donde add_detections usa la alternancia
            // de marcha (TrackLeg::raw_speed()) para desempatar en vez de
            // fiarse de una diferencia de distancia insignificante.
            static constexpr double AMBIGUITY_MARGIN_M = 0.08;
    };




#endif
