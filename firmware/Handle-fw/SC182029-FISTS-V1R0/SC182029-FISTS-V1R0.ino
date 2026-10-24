/* ================================================================

   Proyecto: Modulo de empuñadura en andador para UMA

   Placa de desarrollo: 
            Arduino Nano ATMega328 TQFP
   Función: Mide el peso ejercido sobre la empuñadura del andador
            y recibe/envía los datos a través del USB

   Versión inicial:
      Autor: Justo Barroso Fontalba
      Fecha: 14/03/2019
   Versión actual:   
      Autor: Manuel Fernández Carmona
      Fecha: 17/11/2021

   Cambios (2026-10, sin probar en la placa: compilar y comprobar con `ros2 topic hz`, ver ../README.md):
      - Se quita el power_down()/power_up() del HX711 en cada lectura. Tras despertar, el HX711
        tarda ~400 ms en dar un dato estable (10 SPS), de ahí los ~2.4 Hz que se medían en las
        manetas. Ahora se envía una línea por cada conversión del HX711 (10 Hz con el pin RATE a
        masa, 80 Hz con RATE a VCC), sin bloquear el bucle esperando al dato.
      - AVERAGE_SAMPLES: promedia varias conversiones por línea (por defecto 1).
      - El cero (tara) espera a que el HX711 se estabilice, descarta las primeras conversiones y
        promedia TARE_SAMPLES (antes 10). Con la maneta SIN carga al arrancar.
      - Órdenes por el puerto serie (solo si alguien las envía; el nodo de ROS no envía nada):
        'T' repite la tara con la maneta descargada, 'S' imprime el estado. Las respuestas empiezan
        por '#' y NUNCA se envían sin pedirlas: el nodo de ROS lee todas las líneas como
        "<preámbulo><número>" y una línea de texto lo haría fallar.
      - El formato de las líneas de datos no cambia ("LL:12.34\n"), ni HX711_SCALE: la tabla de
        calibración de walker_loads/config/handle_calib.yaml sigue valiendo.

   ================================================================ */

/* ================================================================
  Librerías
   ================================================================ */

#include <SPI.h>
// Usamos la version 0.3 de ROB TILLARD
#include <HX711.h>


/* ================================================================
   Definición de pines
   ================================================================ */

//Arduino Nano ATMega328 
#define HX711_DATA      3
#define HX711_SCK       2

/* ================================================================
   Definición de constantes
   ================================================================ */

#define HANDLE_SIDE  "LL:"
//#define HANDLE_SIDE  "RL:"
#define END_CHAR  "\n"
#define HX711_SCALE 2280.f // Este valor es obtenido de calibrar la escala con diferentes pesos
#define SERIAL_BAUDRATE 115200
#define AVERAGE_SAMPLES 1   // conversiones del HX711 que se promedian en cada línea enviada
#define TARE_SAMPLES    20  // conversiones que se promedian para fijar el cero
#define SETTLE_MS       600 // espera tras arrancar el HX711, antes de la tara (400 ms a 10 SPS)

/* ================================================================
   Definición de funciones
   ================================================================ */

void tareLoadcell();      // Fija el cero con la maneta sin carga
void readCommands();      // Atiende las órdenes 'T' y 'S' del puerto serie
void printStatus();       // Responde a 'S'

/* ================================================================
   Definición de estructuras
   ================================================================ */

/* ================================================================
   Drivers
   ================================================================ */

HX711 loadcell;
float sum = 0.f;                       // suma de las conversiones pendientes de enviar
uint8_t pending = 0;                   // cuántas hay
unsigned long conversions = 0;         // conversiones leídas desde la última 'S'
unsigned long statusStartMs = 0;
/* ================================================================
   Inicialización
   ================================================================ */

void setup() {
  //Inicialización de puerto serie
  Serial.begin( SERIAL_BAUDRATE );
  //Inicialización de HX711 con el pin de datos como entrada, el pin de reloj como salida y el factor de ganancia (64 o 128 Canal A y 32 Canal B)
  loadcell.begin( HX711_DATA, HX711_SCK );
  loadcell.set_scale(HX711_SCALE);
  delay( SETTLE_MS ); // el HX711 tarda en dar datos estables tras arrancar
  tareLoadcell();
  statusStartMs = millis();
}

/* ================================================================
   Bucle principal
   ================================================================ */

void loop() {
    readCommands();
    // Una línea por conversión del HX711, sin esperar a la siguiente (el chip se queda encendido: cada power_down()/power_up()
    // cuesta ~400 ms de estabilización).
    if ( loadcell.is_ready() ) {
        sum += loadcell.get_units( 1 ); // Peso (unidades de HX711_SCALE) de la conversión que espera
        conversions++;
        if ( ++pending >= AVERAGE_SAMPLES ) {
            Serial.print(HANDLE_SIDE);
            Serial.print(sum / pending);
            Serial.print(END_CHAR);
            sum = 0.f;
            pending = 0;
        }
    }
}

/* ================================================================
   Interrupciones
   ================================================================ */

/* ================================================================
   Funciones
   ================================================================ */

void tareLoadcell() {
  loadcell.read_average( 2 );          // se descartan las primeras conversiones
  loadcell.tare( TARE_SAMPLES );       // el cero es la media de las siguientes
  sum = 0.f;                           // lo que había pendiente era con el cero anterior
  pending = 0;
}

void readCommands() {
  while ( Serial.available() > 0 ) {
    char c = Serial.read();
    if ( c == 'T' ) {
      tareLoadcell();
      Serial.print("#tared\n");
    } else if ( c == 'S' ) {
      printStatus();
    }
  }
}

void printStatus() {
  unsigned long elapsed = millis() - statusStartMs;
  Serial.print("#scale=");
  Serial.print( loadcell.get_scale() );
  Serial.print(" offset=");
  Serial.print( loadcell.get_offset() );
  Serial.print(" conversions_per_s=");
  Serial.print( elapsed > 0 ? 1000.f * conversions / elapsed : 0.f );
  Serial.print(" average=");
  Serial.print( AVERAGE_SAMPLES );
  Serial.print("\n");
  conversions = 0;
  statusStartMs = millis();
}

/* ================================================================
   Fin
   ================================================================ */
