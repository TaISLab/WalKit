/* ================================================================

   Proyecto: Modulo de encoder en andador para UMA
   
   Placa de desarrollo: 
            Arduino Nano ATMega328 TQFP

   Función: Cuenta las vueltas que ha girado la rueda del andador
            en base a la posición del encoder, tiene un led indicador
            y recibe/envía los datos a través del BUS USB

   Versión inicial:
      Autor: Justo Barroso Fontalba
      Fecha: 18/03/2019
   Versión actual:   
      Autor: Manuel Fernández Carmona
      Fecha: 24/11/2021

   ================================================================ */

/* ================================================================
   Librerías
   ================================================================ */

#include <SPI.h>
#include <AS5601.h>

/* ================================================================
   Definición de pines
   ================================================================ */


/* ================================================================
   Definición de funciones
   ================================================================ */


/* ================================================================
   Definición de estructuras
   ================================================================ */

/* ================================================================
   Definición de variables
   ================================================================ */

/* ================================================================
   Definición de constantes
   ================================================================ */

//#define WHEEL_SIDE  "LW:"
#define WHEEL_SIDE    "RW:"
#define END_CHAR  "\n"

// Error codes: still sent as an integer, but in a range out of encoder values
#define ERROR_NO_MAGNET -1
#define ERROR_MAGNET_WEAK -2
#define ERROR_MAGNET_STRONG -3

#define SERIAL_BAUDRATE 115200

/* ================================================================
   Drivers
   ================================================================ */

AS5601 Sensor;

/* ================================================================
   Inicialización
   ================================================================ */

void setup() {
  //Inicialización de puerto serie, velocidad 115200 baudios
  Serial.begin( SERIAL_BAUDRATE );
  //Establecer posición a 0
  Sensor.setZeroPosition();
}

/* ================================================================
   Bucle principal
   ================================================================ */

void loop() {

    // query status register
    unsigned char status = Sensor.readRaw8( AS5601::ByteRegister::STATUS );

    // Is magnet too strong?
    bool isMagnetStrong = bitRead( status, 3 ) == 1 ? true : false;

    // Is magnet too weak?
    bool isMagnetWeak = bitRead( status, 4 ) == 1 ? true : false;

    // Is magnet detected?
    bool isMagnetDetected = bitRead( status, 5 ) == 1 ? true : false;

    Serial.print( WHEEL_SIDE );  
    
    if (!isMagnetDetected){
      Serial.print( ERROR_NO_MAGNET);  
    } else if (isMagnetWeak){
      Serial.print( ERROR_MAGNET_WEAK);  
    } else if (isMagnetStrong){
      Serial.print( ERROR_MAGNET_STRONG);  
    } else if (!isMagnetStrong) && (!isMagnetWeak) && (isMagnetDetected){
    Serial.print( Sensor.getAngle());
    } 

    Serial.print( END_CHAR);  

}

/* ================================================================
   Interrupciones
   ================================================================ */

/* ================================================================
   Funciones
   ================================================================ */

/* ================================================================
   Fin
   ================================================================ */


   
