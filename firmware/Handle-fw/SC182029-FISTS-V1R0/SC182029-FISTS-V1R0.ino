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

/* ================================================================
   Definición de funciones
   ================================================================ */

float readHX711(); //Lee el valor del sensor

/* ================================================================
   Definición de estructuras
   ================================================================ */

/* ================================================================
   Drivers
   ================================================================ */

HX711 loadcell;
float value;
/* ================================================================
   Inicialización
   ================================================================ */

void setup() {
  //Inicialización de puerto serie
  Serial.begin( SERIAL_BAUDRATE );
  //Inicialización de HX711 con el pin de datos como entrada, el pin de reloj como salida y el factor de ganancia (64 o 128 Canal A y 32 Canal B)
  loadcell.begin( HX711_DATA, HX711_SCK );
  loadcell.set_scale(HX711_SCALE);
  loadcell.tare(); //Establece la escala en 0
}

/* ================================================================
   Bucle principal
   ================================================================ */

void loop() {
    value = readHX711(); // Llamada a función para obtención del valor del peso
    Serial.print(HANDLE_SIDE);
    Serial.print(value);
    Serial.print(END_CHAR);
}

/* ================================================================
   Interrupciones
   ================================================================ */

/* ================================================================
   Funciones
   ================================================================ */

float readHX711() {
  static float loadcellValue;
  loadcell.power_up(); // Célula activa
  loadcellValue = loadcell.get_units();
  loadcell.power_down(); // Célula en reposo
  return loadcellValue;
}

/* ================================================================
   Fin
   ================================================================ */
